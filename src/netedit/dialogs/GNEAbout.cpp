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
/// @file    GNEAbout.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// The "About" - dialog for netedit, (adapted from GUIDialog_AboutSUMO)
/****************************************************************************/
#include <config.h>

#ifdef HAVE_VERSION_H
#include <version.h>
#endif

#include <utils/common/MsgHandler.h>
#include <utils/foxtools/MFXLinkLabel.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/div/GUIDesigns.h>

#include "GNEAbout.h"


// ===========================================================================
// method definitions
// ===========================================================================
GNEAbout::GNEAbout(FXWindow* parent) :
    FXDialogBox(parent, TL("About Eclipse SUMO netedit"), GUIDesignDialogBox) {
    // set dialog icon
    setIcon(GUIIconSubSys::getIcon(GUIIcon::NETEDIT_MINI));

    // create frame for main info
    FXHorizontalFrame* mainInfoFrame = new FXHorizontalFrame(this, GUIDesignAuxiliarHorizontalFrame);

    // Netedit icon
    new FXLabel(mainInfoFrame, "", GUIIconSubSys::getIcon(GUIIcon::SUMO_LOGO), GUIDesignLabelIcon);

    // "SUMO <VERSION>"
    FXVerticalFrame* descriptionFrame = new FXVerticalFrame(mainInfoFrame, GUIDesignLabelAboutInfo);
    myHeadlineFont = new FXFont(getApp(), "Arial", 18, FXFont::Bold);
    FXLabel* neteditLabel = new FXLabel(descriptionFrame, "SUMO netedit " VERSION_STRING, nullptr, GUIDesignLabelAboutInfo);
    neteditLabel->setFont(myHeadlineFont);
    new FXLabel(descriptionFrame, TL("Network editor for Eclipse SUMO, the Simulation of Urban MObility"), nullptr, GUIDesignLabelAboutInfo);
    new FXLabel(descriptionFrame, TL("Graphical editor for road networks and infrastructure."), nullptr, GUIDesignLabelAboutInfo);
    // show modules
    new FXLabel(descriptionFrame, HAVE_ENABLED, nullptr, GUIDesignLabelAboutInfo);
    // write HAVE_ENABLED with the current modules (except Windows) in debug mode
    std::string modules(HAVE_ENABLED);
    while ((modules.size() > 0) && (modules.front() != ' ')) {
        modules.erase(modules.begin());
    }
    WRITE_DEBUG(("Modules: " + modules).c_str());
    // SUMO_HOME
    new FXLabel(descriptionFrame, std::string("SUMO_HOME: " + std::string(getenv("SUMO_HOME"))).c_str(), nullptr, GUIDesignLabelAboutInfo);
    // copyright notice
    new FXLabel(this, "Copyright (C) 2001-2024 German Aerospace Center (DLR) and others.", nullptr, GUIDesignLabelAboutInfo);
    new FXLabel(this, TL("This application is based on code provided by the Eclipse SUMO project."), nullptr, GUIDesignLabelAboutInfo);
    new FXLabel(this, TL("These core components are available under the conditions of the Eclipse Public License v2."), nullptr, GUIDesignLabelAboutInfo);
    (new MFXLinkLabel(this, "SPDX-License-Identifier: EPL-2.0", nullptr, GUIDesignLabelAboutInfo))->setTipText("https://www.eclipse.org/legal/epl-v20.html");

    // link to homepage
    (new MFXLinkLabel(this, "https://www.eclipse.dev/sumo", nullptr, GUIDesignLabel(JUSTIFY_NORMAL)))->setTipText("https://www.eclipse.dev/sumo");

    // centered ok-button
    FXHorizontalFrame* buttonFrame = new FXHorizontalFrame(this, GUIDesignHorizontalFrame);
    new FXHorizontalFrame(buttonFrame, GUIDesignAuxiliarHorizontalFrame);
    FXButton* OKButton = GUIDesigns::buildFXButton(buttonFrame, TL("&OK"), "", "", GUIIconSubSys::getIcon(GUIIcon::ACCEPT), this, ID_ACCEPT, GUIDesignButtonOK);
    new FXHorizontalFrame(buttonFrame, GUIDesignAuxiliarHorizontalFrame);

    // focus OK button
    OKButton->setFocus();
}


void
GNEAbout::create() {
    FXDialogBox::create();
}


GNEAbout::~GNEAbout() {
    delete myHeadlineFont;
}


/****************************************************************************/
