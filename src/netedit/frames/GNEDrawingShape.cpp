/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2024 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    GNEDrawingShape.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2022
///
// Frame for draw shapes
/****************************************************************************/
#include <config.h>

#include <netedit/frames/common/GNEInspectorFrame.h>
#include <netedit/GNEViewNet.h>
#include <utils/foxtools/MFXDynamicLabel.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>

#include "GNEDrawingShape.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEDrawingShape) DrawingShapeMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_STARTDRAWING,   GNEDrawingShape::onCmdStartDrawing),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_STOPDRAWING,    GNEDrawingShape::onCmdStopDrawing),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ABORTDRAWING,   GNEDrawingShape::onCmdAbortDrawing)
};

// Object implementation
FXIMPLEMENT(GNEDrawingShape,               MFXGroupBoxModule,     DrawingShapeMap,                ARRAYNUMBER(DrawingShapeMap))


// ===========================================================================
// method definitions
// ===========================================================================

GNEDrawingShape::GNEDrawingShape(GNEFrame* frameParent) :
    MFXGroupBoxModule(frameParent, TL("Drawing")),
    myFrameParent(frameParent),
    myDeleteLastCreatedPoint(false) {
    // create start and stop buttons
    myStartDrawingButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Start drawing"), "", "", 0, this, MID_GNE_STARTDRAWING, GUIDesignButton);
    myStopDrawingButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Stop drawing"), "", "", 0, this, MID_GNE_STOPDRAWING, GUIDesignButton);
    myAbortDrawingButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Abort drawing"), "", "", 0, this, MID_GNE_ABORTDRAWING, GUIDesignButton);
    // create information label
    std::ostringstream information;
    information
            << "- " << TL("'Start drawing' or ENTER to create a shape.") << "\n"
            << "- " << TL("'Stop drawing' or ENTER to finish shape creation.") << "\n"
            << "- " << TL("'Abort drawing' or ESC to abort shape creation.") << "\n"
            << "- " << TL("'Shift + Click' to remove the last inserted point.");
    myInformationLabel = new MFXDynamicLabel(getCollapsableFrame(), information.str().c_str(), 0, GUIDesignLabelFrameInformation);
    // disable stop and abort functions as init
    myStopDrawingButton->disable();
    myAbortDrawingButton->disable();
}


GNEDrawingShape::~GNEDrawingShape() {}


void GNEDrawingShape::showDrawingShape() {
    // abort current drawing before show
    abortDrawing();
    // show MFXGroupBoxModule
    MFXGroupBoxModule::show();
}


void GNEDrawingShape::hideDrawingShape() {
    // abort current drawing before hide
    abortDrawing();
    // show MFXGroupBoxModule
    MFXGroupBoxModule::hide();
}


void
GNEDrawingShape::startDrawing() {
    // Only start drawing if GNEDrawingShape modul is shown
    if (shown()) {
        // change buttons
        myStartDrawingButton->disable();
        myStopDrawingButton->enable();
        myAbortDrawingButton->enable();
    }
}


void
GNEDrawingShape::stopDrawing() {
    // try to build shape
    if (myFrameParent->shapeDrawed()) {
        // clear created points
        myTemporalShape.clear();
        // change buttons
        myStartDrawingButton->enable();
        myStopDrawingButton->disable();
        myAbortDrawingButton->disable();
    } else {
        // abort drawing if shape cannot be created
        abortDrawing();
    }
}


void
GNEDrawingShape::abortDrawing() {
    // clear created points
    myTemporalShape.clear();
    // change buttons
    myStartDrawingButton->enable();
    myStopDrawingButton->disable();
    myAbortDrawingButton->disable();
}


void
GNEDrawingShape::addNewPoint(const Position& P) {
    if (myStopDrawingButton->isEnabled()) {
        myTemporalShape.push_back(P);
    } else {
        throw ProcessError(TL("A new point cannot be added if drawing wasn't started"));
    }
}


void
GNEDrawingShape::removeLastPoint() {
    if (myTemporalShape.size() > 1) {
        myTemporalShape.pop_back();
    }
}


const PositionVector&
GNEDrawingShape::getTemporalShape() const {
    return myTemporalShape;
}


bool
GNEDrawingShape::isDrawing() const {
    return myStopDrawingButton->isEnabled();
}


void
GNEDrawingShape::setDeleteLastCreatedPoint(bool value) {
    myDeleteLastCreatedPoint = value;
    myFrameParent->getViewNet()->updateViewNet();
}


bool
GNEDrawingShape::getDeleteLastCreatedPoint() {
    return myDeleteLastCreatedPoint;
}


long
GNEDrawingShape::onCmdStartDrawing(FXObject*, FXSelector, void*) {
    startDrawing();
    return 0;
}


long
GNEDrawingShape::onCmdStopDrawing(FXObject*, FXSelector, void*) {
    stopDrawing();
    return 0;
}


long
GNEDrawingShape::onCmdAbortDrawing(FXObject*, FXSelector, void*) {
    abortDrawing();
    return 0;
}

/****************************************************************************/
