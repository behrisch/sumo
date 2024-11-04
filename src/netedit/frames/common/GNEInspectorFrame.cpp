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
/// @file    GNEInspectorFrame.cpp
/// @author  Jakob Erdmann
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2011
///
// The Widget for modifying network-element attributes (i.e. lane speed)
/****************************************************************************/
#include <config.h>

#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEApplicationWindow.h>
#include <netedit/elements/additional/GNERerouter.h>
#include <netedit/elements/additional/GNECalibrator.h>
#include <netedit/elements/additional/GNEVariableSpeedSign.h>
#include <netedit/elements/network/GNEEdgeTemplate.h>
#include <netedit/dialogs/GNEMultipleParametersDialog.h>
#include <netedit/dialogs/GNERerouterDialog.h>
#include <netedit/dialogs/GNECalibratorDialog.h>
#include <netedit/dialogs/GNEVariableSpeedSignDialog.h>
#include <netedit/dialogs/GNESingleParametersDialog.h>
#include <netedit/frames/network/GNECreateEdgeFrame.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>

#include "GNEInspectorFrame.h"
#include "GNEDeleteFrame.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEInspectorFrame) GNEInspectorFrameMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_INSPECTORFRAME_GOBACK,  GNEInspectorFrame::onCmdGoBack)
};

FXDEFMAP(GNEInspectorFrame::NeteditAttributesEditor) NeteditAttributesEditorMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SET_ATTRIBUTE,      GNEInspectorFrame::NeteditAttributesEditor::onCmdSetNeteditAttribute),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_MARKFRONTELEMENT,   GNEInspectorFrame::NeteditAttributesEditor::onCmdMarkFrontElement),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,                   GNEInspectorFrame::NeteditAttributesEditor::onCmdNeteditAttributeHelp)
};

FXDEFMAP(GNEInspectorFrame::GEOAttributesEditor) GEOAttributesEditorMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SET_ATTRIBUTE,  GNEInspectorFrame::GEOAttributesEditor::onCmdSetGEOAttribute),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,               GNEInspectorFrame::GEOAttributesEditor::onCmdGEOAttributeHelp)
};

FXDEFMAP(GNEInspectorFrame::TemplateEditor) TemplateEditorMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_HOTKEY_SHIFT_F1_TEMPLATE_SET,   GNEInspectorFrame::TemplateEditor::onCmdSetTemplate),
    FXMAPFUNC(SEL_COMMAND,  MID_HOTKEY_SHIFT_F2_TEMPLATE_COPY,  GNEInspectorFrame::TemplateEditor::onCmdCopyTemplate),
    FXMAPFUNC(SEL_COMMAND,  MID_HOTKEY_SHIFT_F3_TEMPLATE_CLEAR, GNEInspectorFrame::TemplateEditor::onCmdClearTemplate),
};



FXDEFMAP(GNEInspectorFrame::AdditionalDialog) AdditionalDialogMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_OPEN_ADDITIONAL_DIALOG, GNEInspectorFrame::AdditionalDialog::onCmdOpenAdditionalDialog),
};

// Object implementation
FXIMPLEMENT(GNEInspectorFrame,                          FXVerticalFrame,    GNEInspectorFrameMap,       ARRAYNUMBER(GNEInspectorFrameMap))
FXIMPLEMENT(GNEInspectorFrame::NeteditAttributesEditor, MFXGroupBoxModule,  NeteditAttributesEditorMap, ARRAYNUMBER(NeteditAttributesEditorMap))
FXIMPLEMENT(GNEInspectorFrame::GEOAttributesEditor,     MFXGroupBoxModule,  GEOAttributesEditorMap,     ARRAYNUMBER(GEOAttributesEditorMap))
FXIMPLEMENT(GNEInspectorFrame::TemplateEditor,          MFXGroupBoxModule,  TemplateEditorMap,          ARRAYNUMBER(TemplateEditorMap))
FXIMPLEMENT(GNEInspectorFrame::AdditionalDialog,        MFXGroupBoxModule,  AdditionalDialogMap,        ARRAYNUMBER(AdditionalDialogMap))


// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNEInspectorFrame::NeteditAttributesEditor - methods
// ---------------------------------------------------------------------------

GNEInspectorFrame::NeteditAttributesEditor::NeteditAttributesEditor(GNEInspectorFrame* inspectorFrameParent) :
    MFXGroupBoxModule(inspectorFrameParent, TL("Netedit attributes")),
    myInspectorFrameParent(inspectorFrameParent) {
    // Create mark as front element button
    myMarkFrontElementButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Mark as front element"), "", "", GUIIconSubSys::getIcon(GUIIcon::FRONTELEMENT),
                               this, MID_GNE_MARKFRONTELEMENT, GUIDesignButton);
    // Create elements for parent additional
    myLabelParentAdditional = new FXLabel(getCollapsableFrame(), "Parent", nullptr, GUIDesignLabelThick(JUSTIFY_NORMAL));
    myTextFieldParentAdditional = new FXTextField(getCollapsableFrame(), GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    mySetNewParentButton = new MFXCheckableButton(false, getCollapsableFrame(),
            inspectorFrameParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            "Set new parent", nullptr, this, MID_GNE_SET_ATTRIBUTE, GUIDesignMFXCheckableButton);
    // Create elements for close shape
    myHorizontalFrameCloseShape = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myLabelCloseShape = new FXLabel(myHorizontalFrameCloseShape, "Close shape", nullptr, GUIDesignLabelThickedFixed(100));
    myCheckBoxCloseShape = new FXCheckButton(myHorizontalFrameCloseShape, "", this, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    // Create help button
    myHelpButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Help"), "", "", nullptr, this, MID_HELP, GUIDesignButtonRectangular);
}


GNEInspectorFrame::NeteditAttributesEditor::~NeteditAttributesEditor() {}


void
GNEInspectorFrame::NeteditAttributesEditor::showNeteditAttributesEditor() {
    // get ACs
    const auto& ACs = myInspectorFrameParent->getViewNet()->getInspectedAttributeCarriers();
    // continue if there is edited ACs
    if (ACs.size() > 0) {
        // enable all editable elements
        myTextFieldParentAdditional->enable();
        myCheckBoxCloseShape->enable();
        // obtain tag property (only for improve code legibility)
        const auto& tagValue = ACs.front()->getTagProperty();
        // check if item can be mark as front element
        if (ACs.size() == 1) {
            // show NeteditAttributesEditor
            show();
            // show button
            myMarkFrontElementButton->show();
            // enable or disable
            if (myInspectorFrameParent->getViewNet()->getFrontAttributeCarrier() == ACs.front()) {
                myMarkFrontElementButton->disable();
            } else {
                myMarkFrontElementButton->enable();
            }
        }
        // check if item can block their shape
        if (tagValue.canCloseShape()) {
            // show NeteditAttributesEditor
            show();
            // Iterate over AC to obtain values
            bool value = true;
            for (const auto& inspectedAC : ACs) {
                value &= GNEAttributeCarrier::parse<bool>(inspectedAC->getAttribute(GNE_ATTR_CLOSE_SHAPE));
            }
            // show close shape frame
            myHorizontalFrameCloseShape->show();
            // show help button
            myHelpButton->show();
            // set check box value and update label
            if (value) {
                myCheckBoxCloseShape->setCheck(true);
                myCheckBoxCloseShape->setText("true");
            } else {
                myCheckBoxCloseShape->setCheck(false);
                myCheckBoxCloseShape->setText("false");
            }
        }
        // Check if item has another item as parent and can be reparent
        if (tagValue.isChild() && tagValue.canBeReparent()) {
            // show NeteditAttributesEditor
            show();
            // obtain additional Parent
            std::set<std::string> parents;
            for (const auto& inspectedAC : ACs) {
                parents.insert(inspectedAC->getAttribute(GNE_ATTR_PARENT));
            }
            // show parent additional elements
            myLabelParentAdditional->show();
            myTextFieldParentAdditional->show();
            mySetNewParentButton->show();
            // reset new parent button
            mySetNewParentButton->setChecked(false);
            mySetNewParentButton->setText(TL("Set new parent"));
            // set parent tag icon
            const auto& parentTagProperty = GNEAttributeCarrier::getTagProperty(ACs.front()->getTagProperty().getParentTags().front());
            mySetNewParentButton->setIcon(GUIIconSubSys::getIcon(parentTagProperty.getGUIIcon()));
            // show help button
            myHelpButton->show();
            // set Label and TextField with the Tag and ID of parent
            myLabelParentAdditional->setText((parentTagProperty.getTagStr() + " parent").c_str());
            myTextFieldParentAdditional->setText(toString(parents).c_str());
        }
        // disable all editable elements if we're in demand mode and inspected AC isn't a demand element
        if (GNEFrameAttributeModules::isSupermodeValid(myInspectorFrameParent->getViewNet(), ACs.front()) == false) {
            myTextFieldParentAdditional->disable();
            myCheckBoxCloseShape->disable();
        }
    }
}


void
GNEInspectorFrame::NeteditAttributesEditor::hideNeteditAttributesEditor() {
    // hide all elements of GroupBox
    myLabelParentAdditional->hide();
    myTextFieldParentAdditional->hide();
    mySetNewParentButton->hide();
    myHorizontalFrameCloseShape->hide();
    myMarkFrontElementButton->hide();
    myHelpButton->hide();
    // hide groupbox
    hide();
}


void
GNEInspectorFrame::NeteditAttributesEditor::refreshNeteditAttributesEditor(bool forceRefresh) {
    // get inspected Attribute carriers
    const auto& ACs  = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers();
    if (ACs.size() > 0) {
        // enable or disable mark front element button
        if (myInspectorFrameParent->getViewNet()->getFrontAttributeCarrier() == ACs.front()) {
            myMarkFrontElementButton->disable();
        } else {
            myMarkFrontElementButton->enable();
        }
        // refresh close shape
        if (myHorizontalFrameCloseShape->shown()) {
            // Iterate over AC to obtain values
            bool value = true;
            for (const auto& i : ACs) {
                value &= GNEAttributeCarrier::parse<bool>(i->getAttribute(GNE_ATTR_CLOSE_SHAPE));
            }
            // set check box value and update label
            if (value) {
                myCheckBoxCloseShape->setCheck(true);
                myCheckBoxCloseShape->setText("true");
            } else {
                myCheckBoxCloseShape->setCheck(false);
                myCheckBoxCloseShape->setText("false");
            }
        }
        // Check if item has another item as parent (Currently only for single Additionals)
        if (myTextFieldParentAdditional->shown() && ((myTextFieldParentAdditional->getTextColor() == FXRGB(0, 0, 0)) || forceRefresh)) {
            // set Label and TextField with the Tag and ID of parent
            myLabelParentAdditional->setText((toString(ACs.front()->getTagProperty().getParentTags().front()) + " parent").c_str());
            myTextFieldParentAdditional->setText(ACs.front()->getAttribute(GNE_ATTR_PARENT).c_str());
        }
    }
}


bool
GNEInspectorFrame::NeteditAttributesEditor::isSelectingParent() const {
    if (!shown()) {
        return false;
    } else {
        return (mySetNewParentButton->shown() && mySetNewParentButton->amChecked());
    }
}


void
GNEInspectorFrame::NeteditAttributesEditor::setNewParent(GNEAttributeCarrier* clickedAC) {
    const auto& ACs = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers();
    // check number of inspected ACs
    if ((ACs.size() > 0) && clickedAC) {
        // check parent tags
        for (const auto& parentTag : ACs.front()->getTagProperty().getParentTags()) {
            if (parentTag == clickedAC->getTagProperty().getTag()) {
                // check if we're changing multiple attributes
                if (ACs.size() > 1) {
                    myInspectorFrameParent->myViewNet->getUndoList()->begin(ACs.front(), "Change multiple attributes");
                }
                // replace the parent of all inspected elements
                for (const auto& AC : ACs) {
                    AC->setAttribute(GNE_ATTR_PARENT, clickedAC->getID(), myInspectorFrameParent->myViewNet->getUndoList());
                }
                // finish change multiple attributes
                if (ACs.size() > 1) {
                    myInspectorFrameParent->myViewNet->getUndoList()->end();
                }
                // stop select parent
                stopSelectParent();
                // refresh netedit attributes editor
                refreshNeteditAttributesEditor(true);
            }
        }
    }
}


void
GNEInspectorFrame::NeteditAttributesEditor::stopSelectParent() {
    if (mySetNewParentButton->amChecked()) {
        onCmdSetNeteditAttribute(mySetNewParentButton, 0, nullptr);
    }
}


long
GNEInspectorFrame::NeteditAttributesEditor::onCmdSetNeteditAttribute(FXObject* obj, FXSelector, void*) {
    const auto& ACs = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers();
    // make sure that ACs has elements
    if (ACs.size() > 0) {
        // check if we're changing multiple attributes
        if (ACs.size() > 1) {
            myInspectorFrameParent->myViewNet->getUndoList()->begin(ACs.front(), "Change multiple attributes");
        }
        if (obj == mySetNewParentButton) {
            if (mySetNewParentButton->amChecked()) {
                mySetNewParentButton->setText(TL("Set new parent"));
                mySetNewParentButton->setChecked(false);
            } else {
                mySetNewParentButton->setText(("Click over " + toString(ACs.front()->getTagProperty().getParentTags().front())).c_str());
                mySetNewParentButton->setChecked(true);
            }
        } else if (obj == myCheckBoxCloseShape) {
            // set new values in all inspected Attribute Carriers
            for (const auto& AC : ACs) {
                if (myCheckBoxCloseShape->getCheck() == 1) {
                    AC->setAttribute(GNE_ATTR_CLOSE_SHAPE, "true", myInspectorFrameParent->myViewNet->getUndoList());
                    myCheckBoxCloseShape->setText("true");
                } else {
                    AC->setAttribute(GNE_ATTR_CLOSE_SHAPE, "false", myInspectorFrameParent->myViewNet->getUndoList());
                    myCheckBoxCloseShape->setText("false");
                }
            }
        } else if (obj == myTextFieldParentAdditional) {
            if (ACs.front()->isValid(GNE_ATTR_PARENT, myTextFieldParentAdditional->getText().text())) {
                // replace the parent of all inspected elements
                for (const auto& AC : ACs) {
                    AC->setAttribute(GNE_ATTR_PARENT, myTextFieldParentAdditional->getText().text(), myInspectorFrameParent->myViewNet->getUndoList());
                }
                myTextFieldParentAdditional->setTextColor(FXRGB(0, 0, 0));
                myTextFieldParentAdditional->killFocus();
            } else {
                myTextFieldParentAdditional->setTextColor(FXRGB(255, 0, 0));
            }
        }
        // finish change multiple attributes
        if (ACs.size() > 1) {
            myInspectorFrameParent->myViewNet->getUndoList()->end();
        }
        // force refresh values of AttributesEditor and GEOAttributesEditor
        myInspectorFrameParent->myAttributesEditor->refreshAttributeEditor(true, true);
        myInspectorFrameParent->myGEOAttributesEditor->refreshGEOAttributesEditor(true);
    }
    return 1;
}


long
GNEInspectorFrame::NeteditAttributesEditor::onCmdMarkFrontElement(FXObject*, FXSelector, void*) {
    // check number of elements
    if (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
        // mark AC as front element
        myInspectorFrameParent->getViewNet()->setFrontAttributeCarrier(myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front());
        // disable button
        myMarkFrontElementButton->disable();
    }
    return 1;
}


long
GNEInspectorFrame::NeteditAttributesEditor::onCmdNeteditAttributeHelp(FXObject*, FXSelector, void*) {
    // Create dialog box
    FXDialogBox* additionalNeteditAttributesHelpDialog = new FXDialogBox(getCollapsableFrame(), "Netedit Attributes Help", GUIDesignDialogBox);
    additionalNeteditAttributesHelpDialog->setIcon(GUIIconSubSys::getIcon(GUIIcon::MODEADDITIONAL));
    // set help text
    std::ostringstream help;
    help
            << TL("- Mark as front element: Mark element as front element (Will be drawn over all other elements)\n")
            << TL("- Block movement: disable movement in move mode\n")
            << TL("- Block shape: Disable moving of shape's vertices and edges. Entire shape can be moved'.\n")
            << TL("- Close shape: Add or remove the last vertex with the same position of first edge'.");
    // Create label with the help text
    new FXLabel(additionalNeteditAttributesHelpDialog, help.str().c_str(), nullptr, GUIDesignLabelFrameInformation);
    // Create horizontal separator
    new FXHorizontalSeparator(additionalNeteditAttributesHelpDialog, GUIDesignHorizontalSeparator);
    // Create frame for OK Button
    FXHorizontalFrame* myHorizontalFrameOKButton = new FXHorizontalFrame(additionalNeteditAttributesHelpDialog, GUIDesignAuxiliarHorizontalFrame);
    // Create Button Close (And two more horizontal frames to center it)
    new FXHorizontalFrame(myHorizontalFrameOKButton, GUIDesignAuxiliarHorizontalFrame);
    GUIDesigns::buildFXButton(myHorizontalFrameOKButton, TL("OK"), "", TL("close"), GUIIconSubSys::getIcon(GUIIcon::ACCEPT), additionalNeteditAttributesHelpDialog, FXDialogBox::ID_ACCEPT, GUIDesignButtonOK);
    new FXHorizontalFrame(myHorizontalFrameOKButton, GUIDesignAuxiliarHorizontalFrame);
    // Write Warning in console if we're in testing mode
    WRITE_DEBUG("Opening NeteditAttributesEditor help dialog");
    // create Dialog
    additionalNeteditAttributesHelpDialog->create();
    // show in the given position
    additionalNeteditAttributesHelpDialog->show(PLACEMENT_CURSOR);
    // refresh APP
    getApp()->refresh();
    // open as modal dialog (will block all windows until stop() or stopModal() is called)
    getApp()->runModalFor(additionalNeteditAttributesHelpDialog);
    // Write Warning in console if we're in testing mode
    WRITE_DEBUG("Closing NeteditAttributesEditor help dialog");
    return 1;
}

// ---------------------------------------------------------------------------
// GNEInspectorFrame::GEOAttributesEditor - methods
// ---------------------------------------------------------------------------

GNEInspectorFrame::GEOAttributesEditor::GEOAttributesEditor(GNEInspectorFrame* inspectorFrameParent) :
    MFXGroupBoxModule(inspectorFrameParent, TL("GEO Attributes")),
    myInspectorFrameParent(inspectorFrameParent) {

    // Create Frame for GEOAttribute
    myGEOAttributeFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myGEOAttributeLabel = new FXLabel(myGEOAttributeFrame, "Undefined GEO Attribute", nullptr, GUIDesignLabelThickedFixed(100));
    myGEOAttributeTextField = new FXTextField(myGEOAttributeFrame, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);

    // Create Frame for use GEO
    myUseGEOFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myUseGEOLabel = new FXLabel(myUseGEOFrame, toString(SUMO_ATTR_GEO).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myUseGEOCheckButton = new FXCheckButton(myUseGEOFrame, "false", this, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);

    // Create help button
    myHelpButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Help"), "", "", nullptr, this, MID_HELP, GUIDesignButtonRectangular);
}


GNEInspectorFrame::GEOAttributesEditor::~GEOAttributesEditor() {}


void
GNEInspectorFrame::GEOAttributesEditor::showGEOAttributesEditor() {
    // make sure that ACs has elements
    if (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() > 0) {
        // enable all editable elements
        myGEOAttributeTextField->enable();
        myUseGEOCheckButton->enable();
        // obtain tag property (only for improve code legibility)
        const auto& tagProperty = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->getTagProperty();
        // check if item can use a geo position
        if (tagProperty.hasGEOShape()) {
            // show GEOAttributesEditor
            show();
            // Iterate over AC to obtain values
            bool value = true;
            for (const auto& i : myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers()) {
                value &= GNEAttributeCarrier::parse<bool>(i->getAttribute(SUMO_ATTR_GEO));
            }
            // show use geo frame
            myUseGEOFrame->show();
            // set UseGEOCheckButton value of and update label (only if geo conversion is defined)
            if (GeoConvHelper::getFinal().getProjString() != "!") {
                myUseGEOCheckButton->enable();
                if (value) {
                    myUseGEOCheckButton->setCheck(true);
                    myUseGEOCheckButton->setText("true");
                } else {
                    myUseGEOCheckButton->setCheck(false);
                    myUseGEOCheckButton->setText("false");
                }
            } else {
                myUseGEOCheckButton->disable();
            }
            if (tagProperty.hasGEOShape() && myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
                myGEOAttributeFrame->show();
                myGEOAttributeLabel->setText(toString(SUMO_ATTR_GEOSHAPE).c_str());
                myGEOAttributeTextField->setTextColor(FXRGB(0, 0, 0));
                // only allow edit if geo conversion is defined
                if (GeoConvHelper::getFinal().getProjString() != "!") {
                    myGEOAttributeTextField->enable();
                    myGEOAttributeTextField->setText(myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->getAttribute(SUMO_ATTR_GEOSHAPE).c_str());
                } else {
                    myGEOAttributeTextField->disable();
                    myGEOAttributeTextField->setText(TL("No geo-conversion defined"));
                }
            }
        }
        // disable all editable elements if we're in demand mode and inspected AC isn't a demand element
        if (GNEFrameAttributeModules::isSupermodeValid(myInspectorFrameParent->getViewNet(), myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()) == false) {
            myGEOAttributeTextField->disable();
            myUseGEOCheckButton->disable();
        }
    }
}


void
GNEInspectorFrame::GEOAttributesEditor::hideGEOAttributesEditor() {
    // hide all elements of GroupBox
    myGEOAttributeFrame->hide();
    myUseGEOFrame->hide();
    // hide groupbox
    hide();
}


void
GNEInspectorFrame::GEOAttributesEditor::refreshGEOAttributesEditor(bool forceRefresh) {
    // obtain tag property (only for improve code legibility)
    const auto& tagProperty = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->getTagProperty();
    // Check that myGEOAttributeFrame is shown
    if ((GeoConvHelper::getFinal().getProjString() != "!") && myGEOAttributeFrame->shown() && ((myGEOAttributeTextField->getTextColor() == FXRGB(0, 0, 0)) || forceRefresh)) {
        if (tagProperty.hasGEOShape()) {
            myGEOAttributeTextField->setText(myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->getAttribute(SUMO_ATTR_GEOSHAPE).c_str());
        }
        myGEOAttributeTextField->setTextColor(FXRGB(0, 0, 0));
    }
}


long
GNEInspectorFrame::GEOAttributesEditor::onCmdSetGEOAttribute(FXObject* obj, FXSelector, void*) {
    // make sure that ACs has elements
    if ((GeoConvHelper::getFinal().getProjString() != "!") && (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() > 0)) {
        if (obj == myGEOAttributeTextField) {
            // obtain tag property (only for improve code legibility)
            const auto& tagProperty = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->getTagProperty();
            // Change GEO Attribute depending of type (Position or shape)
            if (tagProperty.hasGEOShape()) {
                if (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->isValid(SUMO_ATTR_GEOSHAPE, myGEOAttributeTextField->getText().text())) {
                    myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front()->setAttribute(SUMO_ATTR_GEOSHAPE, myGEOAttributeTextField->getText().text(), myInspectorFrameParent->myViewNet->getUndoList());
                    myGEOAttributeTextField->setTextColor(FXRGB(0, 0, 0));
                    myGEOAttributeTextField->killFocus();
                } else {
                    myGEOAttributeTextField->setTextColor(FXRGB(255, 0, 0));
                }
            } else {
                throw ProcessError("myGEOAttributeTextField must be hidden because there isn't GEO Attribute to edit");
            }
        } else if (obj == myUseGEOCheckButton) {
            // update GEO Attribute of entire selection
            for (const auto& i : myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers()) {
                if (myUseGEOCheckButton->getCheck() == 1) {
                    i->setAttribute(SUMO_ATTR_GEO, "true", myInspectorFrameParent->myViewNet->getUndoList());
                    myUseGEOCheckButton->setText("true");
                } else {
                    i->setAttribute(SUMO_ATTR_GEO, "false", myInspectorFrameParent->myViewNet->getUndoList());
                    myUseGEOCheckButton->setText("false");
                }
            }
        }
        // force refresh values of Attributes editor and NeteditAttributesEditor
        myInspectorFrameParent->myAttributesEditor->refreshAttributeEditor(true, true);
        myInspectorFrameParent->myNeteditAttributesEditor->refreshNeteditAttributesEditor(true);
    }
    return 1;
}


long
GNEInspectorFrame::GEOAttributesEditor::onCmdGEOAttributeHelp(FXObject*, FXSelector, void*) {
    FXDialogBox* helpDialog = new FXDialogBox(getCollapsableFrame(), "GEO attributes Help", GUIDesignDialogBox);
    std::ostringstream help;
    help
            << TL(" SUMO uses the World Geodetic System 84 (WGS84/UTM).\n")
            << TL(" For a GEO-referenced network, geo coordinates are represented as pairs of Longitude and Latitude\n")
            << TL(" in decimal degrees without extra symbols. (N,W..)\n")
            << TL(" - Longitude: East-west position of a point on the Earth's surface.\n")
            << TL(" - Latitude: North-south position of a point on the Earth's surface.\n")
            << TL(" - CheckBox 'geo' enables or disables saving position in GEO coordinates\n");
    new FXLabel(helpDialog, help.str().c_str(), nullptr, GUIDesignLabelFrameInformation);
    // "OK"
    GUIDesigns::buildFXButton(helpDialog, TL("OK"), "", TL("close"), GUIIconSubSys::getIcon(GUIIcon::ACCEPT), helpDialog, FXDialogBox::ID_ACCEPT, GUIDesignButtonOK);
    helpDialog->create();
    helpDialog->show();
    return 1;
}

// ---------------------------------------------------------------------------
// GNEInspectorFrame::TemplateEditor - methods
// ---------------------------------------------------------------------------

GNEInspectorFrame::TemplateEditor::TemplateEditor(GNEInspectorFrame* inspectorFrameParent) :
    MFXGroupBoxModule(inspectorFrameParent, TL("Templates")),
    myInspectorFrameParent(inspectorFrameParent),
    myEdgeTemplate(nullptr) {
    // Create set template button
    mySetTemplateButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Set as Template"), "", "", nullptr, this, MID_HOTKEY_SHIFT_F1_TEMPLATE_SET, GUIDesignButton);
    // Create copy template button
    myCopyTemplateButton = GUIDesigns::buildFXButton(getCollapsableFrame(), "", "", "", nullptr, this, MID_HOTKEY_SHIFT_F2_TEMPLATE_COPY, GUIDesignButton);
    // Create copy template button
    myClearTemplateButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("clear Edge Template"), "", "", nullptr, this, MID_HOTKEY_SHIFT_F3_TEMPLATE_CLEAR, GUIDesignButton);
}


GNEInspectorFrame::TemplateEditor::~TemplateEditor() {
}


bool
GNEInspectorFrame::TemplateEditor::showTemplateEditor() {
    // show template editor only if we're editing an edge in Network mode AND we have at least one inspected edge
    if (myInspectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        for (const auto& AC : myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers()) {
            if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                // update buttons and show module
                updateButtons();
                show();
                return true;
            }
        }
    }
    return false;
}


void
GNEInspectorFrame::TemplateEditor::hideTemplateEditor() {
    // hide template editor
    hide();
}


GNEEdgeTemplate*
GNEInspectorFrame::TemplateEditor::getEdgeTemplate() const {
    return myEdgeTemplate;
}


void
GNEInspectorFrame::TemplateEditor::setEdgeTemplate(const GNEEdge* edge) {
    // delete previous template edge
    if (myEdgeTemplate) {
        delete myEdgeTemplate;
        myEdgeTemplate = nullptr;
    }
    // update edge template
    if (edge) {
        myEdgeTemplate = new GNEEdgeTemplate(edge);
        // use template by default
        myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getViewParent()->getCreateEdgeFrame()->setUseEdgeTemplate();
    }
}


void
GNEInspectorFrame::TemplateEditor::updateEdgeTemplate() {
    if (myEdgeTemplate) {
        myEdgeTemplate->updateLaneTemplates();
        // use template by default
        myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getViewParent()->getCreateEdgeFrame()->setUseEdgeTemplate();
    }
}

void
GNEInspectorFrame::TemplateEditor::setTemplate() {
    // check if template editor AND mySetTemplateButton is enabled
    if (shown() && mySetTemplateButton->isEnabled()) {
        onCmdSetTemplate(nullptr, 0, nullptr);
    }
}


void
GNEInspectorFrame::TemplateEditor::copyTemplate() {
    // check if template editor AND myCopyTemplateButton is enabled
    if (shown() && myCopyTemplateButton->isEnabled()) {
        onCmdCopyTemplate(nullptr, 0, nullptr);
    }
}


void
GNEInspectorFrame::TemplateEditor::clearTemplate() {
    // check if template editor AND myClearTemplateButton is enabled
    if (shown() && myClearTemplateButton->isEnabled()) {
        onCmdClearTemplate(nullptr, 0, nullptr);
    }
}


long
GNEInspectorFrame::TemplateEditor::onCmdSetTemplate(FXObject*, FXSelector, void*) {
    // apply to all selected edges
    for (const auto& AC : myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers()) {
        if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
            // set template
            setEdgeTemplate(myInspectorFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(AC->getID()));
            // update buttons
            updateButtons();
        }
    }
    return 1;
}


long
GNEInspectorFrame::TemplateEditor::onCmdCopyTemplate(FXObject*, FXSelector, void*) {
    // first check
    if (myEdgeTemplate) {
        // begin copy template
        myInspectorFrameParent->myViewNet->getUndoList()->begin(myEdgeTemplate, "copy edge template");
        // iterate over inspected ACs
        for (const auto& inspectedAC : myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers()) {
            // avoid copy template in the same edge
            if (inspectedAC->getID() != myEdgeTemplate->getID()) {
                // retrieve edge ID (and throw exception if edge doesn't exist)
                myInspectorFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(inspectedAC->getID())->copyTemplate(myEdgeTemplate, myInspectorFrameParent->myViewNet->getUndoList());
            }
        }
        // end copy template
        myInspectorFrameParent->myViewNet->getUndoList()->end();
        // refresh inspector parent
        myInspectorFrameParent->myAttributesEditor->refreshAttributeEditor(true, true);
    }
    return 1;
}


long
GNEInspectorFrame::TemplateEditor::onCmdClearTemplate(FXObject*, FXSelector, void*) {
    // set null edge
    setEdgeTemplate(nullptr);
    // update buttons
    updateButtons();
    return 1;
}


void
GNEInspectorFrame::TemplateEditor::updateButtons() {
    const auto& inspectedACs = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers();
    // only show set template button if we have exactly one inspected edge
    if (inspectedACs.size() == 1) {
        mySetTemplateButton->setText((TLF("Set edge '%' as Template", inspectedACs.front()->getID())).c_str());
        mySetTemplateButton->show();
    } else {
        mySetTemplateButton->hide();
    }
    // enable or disable clear buttons depending of myEdgeTemplate
    if (myEdgeTemplate) {
        // update caption of copy button
        if (inspectedACs.size() == 1) {
            myCopyTemplateButton->setText(("Copy '" + myEdgeTemplate->getID() + "' into edge '" + inspectedACs.front()->getID() + "'").c_str());
        } else {
            myCopyTemplateButton->setText(("Copy '" + myEdgeTemplate->getID() + "' into " + toString(inspectedACs.size()) + " selected edges").c_str());
        }
        // enable set and clear buttons
        myCopyTemplateButton->enable();
        myClearTemplateButton->enable();
    } else {
        // update caption of copy button
        myCopyTemplateButton->setText(TL("No edge Template Set"));
        // disable set and clear buttons
        myCopyTemplateButton->disable();
        myClearTemplateButton->disable();
    }
}

// ---------------------------------------------------------------------------
// GNEInspectorFrame::AdditionalDialog - methods
// ---------------------------------------------------------------------------

GNEInspectorFrame::AdditionalDialog::AdditionalDialog(GNEInspectorFrame* inspectorFrameParent) :
    MFXGroupBoxModule(inspectorFrameParent, TL("Additional dialog")),
    myInspectorFrameParent(inspectorFrameParent) {
    // Create mark as front element button
    myOpenAdditionalDialog = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Additional dialog"), "", "", nullptr, this, MID_OPEN_ADDITIONAL_DIALOG, GUIDesignButton);
}


GNEInspectorFrame::AdditionalDialog::~AdditionalDialog() {}


void
GNEInspectorFrame::AdditionalDialog::showAdditionalDialog() {
    // check number of inspected elements
    if (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
        // get AC
        const GNEAttributeCarrier* AC = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front();
        // check AC
        if (AC->getTagProperty().getTag() == SUMO_TAG_REROUTER) {
            // update button
            myOpenAdditionalDialog->setText(TL("Open rerouter dialog"));
            myOpenAdditionalDialog->setIcon(GUIIconSubSys::getIcon(GUIIcon::REROUTER));
            // show module
            show();
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_CALIBRATOR) {
            // update button
            myOpenAdditionalDialog->setText(TL("Open calibrator dialog"));
            myOpenAdditionalDialog->setIcon(GUIIconSubSys::getIcon(GUIIcon::CALIBRATOR));
            // show module
            show();
        } else if (AC->getTagProperty().getTag() == GNE_TAG_CALIBRATOR_LANE) {
            // update button
            myOpenAdditionalDialog->setText(TL("Open calibrator lane dialog"));
            myOpenAdditionalDialog->setIcon(GUIIconSubSys::getIcon(GUIIcon::CALIBRATOR));
            // show module
            show();
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_VSS) {
            // update button
            myOpenAdditionalDialog->setText(TL("Open VSS dialog"));
            myOpenAdditionalDialog->setIcon(GUIIconSubSys::getIcon(GUIIcon::VARIABLESPEEDSIGN));
            // show module
            show();
        }
    } else {
        // hide module
        hide();
    }
}


void
GNEInspectorFrame::AdditionalDialog::hideAdditionalDialog() {
    // hide groupbox
    hide();
}


long
GNEInspectorFrame::AdditionalDialog::onCmdOpenAdditionalDialog(FXObject*, FXSelector, void*) {
    // check number of inspected elements
    if (myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
        // get AC
        GNEAttributeCarrier* AC = myInspectorFrameParent->myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().front();
        // check AC
        if (AC->getTagProperty().getTag() == SUMO_TAG_REROUTER) {
            // Open rerouter dialog
            GNERerouterDialog(dynamic_cast<GNERerouter*>(AC));
        } else if ((AC->getTagProperty().getTag() == SUMO_TAG_CALIBRATOR) || (AC->getTagProperty().getTag() == GNE_TAG_CALIBRATOR_LANE)) {
            // Open calibrator dialog
            GNECalibratorDialog(dynamic_cast<GNECalibrator*>(AC));
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_VSS) {
            // Open VSS dialog
            GNEVariableSpeedSignDialog(dynamic_cast<GNEVariableSpeedSign*>(AC));
        }
    }
    return 1;
}

// ---------------------------------------------------------------------------
// GNEInspectorFrame - methods
// ---------------------------------------------------------------------------

GNEInspectorFrame::GNEInspectorFrame(GNEViewParent* viewParent, GNEViewNet* viewNet) :
    GNEFrame(viewParent, viewNet, "Inspector"),
    myPreviousElementInspect(nullptr),
    myPreviousElementDelete(nullptr),
    myPreviousElementDeleteWasMarked(false) {

    // Create back button
    myBackButton = GUIDesigns::buildFXButton(myHeaderLeftFrame, "", "", "", GUIIconSubSys::getIcon(GUIIcon::BIGARROWLEFT), this, MID_GNE_INSPECTORFRAME_GOBACK, GUIDesignButtonRectangular);
    myHeaderLeftFrame->hide();
    myBackButton->hide();

    // Create Overlapped Inspection module
    myOverlappedInspection = new GNEOverlappedInspection(this);

    // Create Attributes Editor module
    myAttributesEditor = new GNEFrameAttributeModules::AttributesEditor(this);

    // Create GEO Parameters Editor module
    myGEOAttributesEditor = new GEOAttributesEditor(this);

    // create parameters Editor module
    myParametersEditor = new GNEFrameAttributeModules::ParametersEditor(this);

    // create additional dialog
    myAdditionalDialog = new AdditionalDialog(this);

    // Create Netedit Attributes Editor module
    myNeteditAttributesEditor = new NeteditAttributesEditor(this);

    // Create Template editor module
    myTemplateEditor = new TemplateEditor(this);

    // Create GNEElementTree module
    myHierarchicalElementTree = new GNEElementTree(this);
}


GNEInspectorFrame::~GNEInspectorFrame() {}


void
GNEInspectorFrame::show() {
    // inspect a null element to reset inspector frame
    inspectSingleElement(nullptr);
    // stop select new element
    myNeteditAttributesEditor->stopSelectParent();
    // show
    GNEFrame::show();
}


void
GNEInspectorFrame::hide() {
    myViewNet->setInspectedAttributeCarriers({});
    GNEFrame::hide();
}


bool
GNEInspectorFrame::processNetworkSupermodeClick(const Position& clickedPosition, GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get unlocked attribute carrier front
    auto AC = viewObjects.getAttributeCarrierFront();
    // first check if we have clicked over an Attribute Carrier
    if (AC) {
        // if Control key is Pressed, select instead inspect element
        if (myViewNet->getMouseButtonKeyPressed().controlKeyPressed()) {
            // toggle networkElement selection
            if (AC->isAttributeCarrierSelected()) {
                AC->unselectAttributeCarrier();
            } else {
                AC->selectAttributeCarrier();
            }
        } else {
            // first check if we clicked over a GNEOverlappedInspection point
            if (myViewNet->getMouseButtonKeyPressed().shiftKeyPressed()) {
                if (!myOverlappedInspection->previousElement(clickedPosition)) {
                    // inspect attribute carrier, (or multiselection if AC is selected)
                    inspectClickedElement(viewObjects, clickedPosition);
                }
            } else if (!myOverlappedInspection->nextElement(clickedPosition)) {
                // inspect attribute carrier, (or multiselection if AC is selected)
                inspectClickedElement(viewObjects, clickedPosition);
            }
            // focus upper element of inspector frame
            focusUpperElement();
        }
        return true;
    } else {
        return false;
    }
}


bool
GNEInspectorFrame::processDemandSupermodeClick(const Position& clickedPosition, GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get unlocked attribute carrier front
    auto AC = viewObjects.getAttributeCarrierFront();
    // first check if we have clicked over a demand element
    if (AC) {
        // if Control key is Pressed, select instead inspect element
        if (myViewNet->getMouseButtonKeyPressed().controlKeyPressed()) {
            // toggle networkElement selection
            if (AC->isAttributeCarrierSelected()) {
                AC->unselectAttributeCarrier();
            } else {
                AC->selectAttributeCarrier();
            }
        } else {
            // first check if we clicked over a GNEOverlappedInspection point
            if (myViewNet->getMouseButtonKeyPressed().shiftKeyPressed()) {
                if (!myOverlappedInspection->previousElement(clickedPosition)) {
                    // inspect attribute carrier, (or multiselection if AC is selected)
                    inspectClickedElement(viewObjects, clickedPosition);
                }
            } else  if (!myOverlappedInspection->nextElement(clickedPosition)) {
                // inspect attribute carrier, (or multiselection if AC is selected)
                inspectClickedElement(viewObjects, clickedPosition);
            }
            // focus upper element of inspector frame
            focusUpperElement();
        }
        return true;
    } else {
        return false;
    }
}


bool
GNEInspectorFrame::processDataSupermodeClick(const Position& clickedPosition, GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get unlocked attribute carrier front
    auto AC = viewObjects.getAttributeCarrierFront();
    // first check if we have clicked over a data element
    if (AC) {
        // if Control key is Pressed, select instead inspect element
        if (myViewNet->getMouseButtonKeyPressed().controlKeyPressed()) {
            // toggle networkElement selection
            if (AC->isAttributeCarrierSelected()) {
                AC->unselectAttributeCarrier();
            } else {
                AC->selectAttributeCarrier();
            }
        } else {
            // first check if we clicked over a GNEOverlappedInspection point
            if (myViewNet->getMouseButtonKeyPressed().shiftKeyPressed()) {
                if (!myOverlappedInspection->previousElement(clickedPosition)) {
                    // inspect attribute carrier, (or multiselection if AC is selected)
                    inspectClickedElement(viewObjects, clickedPosition);
                }
            } else  if (!myOverlappedInspection->nextElement(clickedPosition)) {
                // inspect attribute carrier, (or multiselection if AC is selected)
                inspectClickedElement(viewObjects, clickedPosition);
            }
            // focus upper element of inspector frame
            focusUpperElement();
        }
        return true;
    } else {
        return false;
    }
}


void
GNEInspectorFrame::inspectSingleElement(GNEAttributeCarrier* AC) {
    // Use the implementation of inspect for multiple AttributeCarriers to avoid repetition of code
    std::vector<GNEAttributeCarrier*> itemsToInspect;
    if (AC != nullptr) {
        myViewNet->setInspectedAttributeCarriers({AC});
        if (AC->isAttributeCarrierSelected() && !myViewNet->getMouseButtonKeyPressed().altKeyPressed()) {
            // obtain selected ACs depending of current supermode
            const auto selectedACs = myViewNet->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(false);
            // iterate over selected ACs
            for (const auto& selectedAC : selectedACs) {
                // filter ACs to inspect using Tag as criterion
                if (selectedAC->getTagProperty().getTag() == AC->getTagProperty().getTag()) {
                    itemsToInspect.push_back(selectedAC);
                }
            }
        } else {
            itemsToInspect.push_back(AC);
        }
    }
    inspectMultisection(itemsToInspect);
}


void
GNEInspectorFrame::inspectMultisection(const std::vector<GNEAttributeCarrier*>& ACs) {
    // update inspected ACs in viewNet
    myViewNet->setInspectedAttributeCarriers(ACs);
    // hide back button
    myHeaderLeftFrame->hide();
    myBackButton->hide();
    // Hide all elements
    myAttributesEditor->hideAttributesEditorModule();
    myNeteditAttributesEditor->hideNeteditAttributesEditor();
    myGEOAttributesEditor->hideGEOAttributesEditor();
    myParametersEditor->hideParametersEditor();
    myAdditionalDialog->hideAdditionalDialog();
    myTemplateEditor->hideTemplateEditor();
    myHierarchicalElementTree->hideHierarchicalElementTree();
    myOverlappedInspection->hideOverlappedInspection();
    // If vector of attribute Carriers contain data
    if (ACs.size() > 0) {
        // Set header
        std::string headerString;
        if (ACs.front()->getTagProperty().isNetworkElement()) {
            headerString = "Net: ";
        } else if (ACs.front()->getTagProperty().isAdditionalElement()) {
            headerString = "Additional: ";
        } else if (ACs.front()->getTagProperty().isShapeElement()) {
            headerString = "Shape: ";
        } else if (ACs.front()->getTagProperty().isTAZElement()) {
            headerString = "TAZ: ";
        } else if (ACs.front()->getTagProperty().isWireElement()) {
            headerString = "WIRE: ";
        } else if (ACs.front()->getTagProperty().isVehicle()) {
            headerString = "Vehicle: ";
        } else if (ACs.front()->getTagProperty().isRoute()) {
            headerString = "Route: ";
        } else if (ACs.front()->getTagProperty().isPerson()) {
            headerString = "Person: ";
        } else if (ACs.front()->getTagProperty().isPlanPerson()) {
            headerString = "PersonPlan: ";
        } else if (ACs.front()->getTagProperty().isContainer()) {
            headerString = "Container: ";
        } else if (ACs.front()->getTagProperty().isPlanContainer()) {
            headerString = "ContainerPlan: ";
        } else if (ACs.front()->getTagProperty().isVehicleStop()) {
            headerString = "Stop: ";
        } else if (ACs.front()->getTagProperty().isDataElement()) {
            headerString = "Data: ";
        }
        if (ACs.size() > 1) {
            headerString += toString(ACs.size()) + " ";
        }
        headerString += ACs.front()->getTagStr();
        if (ACs.size() > 1) {
            headerString += "s";
        }
        // Set headerString into header label
        getFrameHeaderLabel()->setText(headerString.c_str());

        // Show attributes editor
        myAttributesEditor->showAttributeEditorModule(true);

        // show netedit attributes editor if  we're inspecting elements with Netedit Attributes
        myNeteditAttributesEditor->showNeteditAttributesEditor();

        // Show GEO Attributes Editor if we're inspecting elements with GEO Attributes
        myGEOAttributesEditor->showGEOAttributesEditor();

        // show parameters editor
        myParametersEditor->showParametersEditor();

        // show additional dialog
        myAdditionalDialog->showAdditionalDialog();

        // If attributes correspond to an Edge and we aren't in demand mode, show template editor
        myTemplateEditor->showTemplateEditor();

        // if we inspect a single Attribute carrier vector, show their children
        if (ACs.size() == 1) {
            myHierarchicalElementTree->showHierarchicalElementTree(ACs.front());
        }
    } else {
        getFrameHeaderLabel()->setText(TL("Inspect"));
    }
    // update frame width
    setFrameWidth(myViewNet->getViewParent()->getFrameAreaWidth());
    // update viewNet
    myViewNet->update();
}


void
GNEInspectorFrame::inspectChild(GNEAttributeCarrier* AC, GNEAttributeCarrier* previousElement) {
    // Show back button if myPreviousElementInspect was defined
    myPreviousElementInspect = previousElement;
    if (myPreviousElementInspect != nullptr) {
        // disable myPreviousElementDelete to avoid inconsistences
        myPreviousElementDelete = nullptr;
        inspectSingleElement(AC);
        myHeaderLeftFrame->show();
        myBackButton->show();
    }
}


void
GNEInspectorFrame::inspectFromDeleteFrame(GNEAttributeCarrier* AC, GNEAttributeCarrier* previousElement, bool previousElementWasMarked) {
    myPreviousElementDelete = previousElement;
    myPreviousElementDeleteWasMarked = previousElementWasMarked;
    // Show back button if myPreviousElementDelete is valid
    if (myPreviousElementDelete != nullptr) {
        // disable myPreviousElementInspect to avoid inconsistences
        myPreviousElementInspect = nullptr;
        inspectSingleElement(AC);
        myHeaderLeftFrame->show();
        myBackButton->show();
    }
}


void
GNEInspectorFrame::clearInspectedAC() {
    // Only remove if there is inspected ACs
    if (myAttributesEditor->getFrameParent()->getViewNet()->getInspectedAttributeCarriers().size() > 0) {
        myViewNet->setInspectedAttributeCarriers({ nullptr });
        // Inspect empty selection (to hide all Editors)
        inspectMultisection({});
    }
}


GNEFrameAttributeModules::AttributesEditor*
GNEInspectorFrame::getAttributesEditor() const {
    return myAttributesEditor;
}


GNEInspectorFrame::NeteditAttributesEditor*
GNEInspectorFrame::getNeteditAttributesEditor() const {
    return myNeteditAttributesEditor;
}


GNEInspectorFrame::TemplateEditor*
GNEInspectorFrame::getTemplateEditor() const {
    return myTemplateEditor;
}


GNEOverlappedInspection*
GNEInspectorFrame::getOverlappedInspection() const {
    return myOverlappedInspection;
}


GNEElementTree*
GNEInspectorFrame::getHierarchicalElementTree() const {
    return myHierarchicalElementTree;
}


long
GNEInspectorFrame::onCmdGoBack(FXObject*, FXSelector, void*) {
    // Inspect previous element or go back to Delete Frame
    if (myPreviousElementInspect) {
        inspectSingleElement(myPreviousElementInspect);
        myPreviousElementInspect = nullptr;
    } else if (myPreviousElementDelete != nullptr) {
        myPreviousElementDelete = nullptr;
        // Hide inspect frame and show delete frame
        hide();
        myViewNet->getViewParent()->getDeleteFrame()->show();
    }
    return 1;
}


void
GNEInspectorFrame::updateFrameAfterUndoRedo() {
    // refresh Attribute Editor
    myAttributesEditor->refreshAttributeEditor(false, false);
    // refresh parametersEditor
    myParametersEditor->refreshParametersEditor();
    // refresh AC Hierarchy
    myHierarchicalElementTree->refreshHierarchicalElementTree();
}


void
GNEInspectorFrame::selectedOverlappedElement(GNEAttributeCarrier* AC) {
    inspectSingleElement(AC);
    // update view (due dotted contour)
    myViewNet->updateViewNet();
}


void
GNEInspectorFrame::inspectClickedElement(const GNEViewNetHelper::ViewObjectsSelector& viewObjects, const Position& clickedPosition) {
    // get front unlocked AC
    const auto AC = viewObjects.getAttributeCarrierFront();
    // check if selection is blocked
    if (AC) {
        // inspect front element
        inspectSingleElement(AC);
        // show Overlapped Inspection module
        myOverlappedInspection->showOverlappedInspection(viewObjects, clickedPosition);
    }
}


void
GNEInspectorFrame::attributeUpdated(SumoXMLAttr /*attribute*/) {
    myAttributesEditor->refreshAttributeEditor(false, false);
    myNeteditAttributesEditor->refreshNeteditAttributesEditor(true);
    myGEOAttributesEditor->refreshGEOAttributesEditor(true);
}



/****************************************************************************/
