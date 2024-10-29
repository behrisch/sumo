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
/// @file    GNEGenericDataFrame.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jan 2020
///
// The Widget for add genericData elements
/****************************************************************************/
#include <config.h>

#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/elements/data/GNEDataHandler.h>
#include <netedit/elements/data/GNEDataInterval.h>
#include <utils/common/MsgHandler.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>

#include "GNEGenericDataFrame.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEGenericDataFrame::DataSetSelector) DataSetSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CREATE,              GNEGenericDataFrame::DataSetSelector::onCmdCreateDataSet),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATASET_NEW,         GNEGenericDataFrame::DataSetSelector::onCmdSetNewDataSetID),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATASET_SELECTED,    GNEGenericDataFrame::DataSetSelector::onCmdSelectDataSet),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SELECT,              GNEGenericDataFrame::DataSetSelector::onCmdSelectCheckButton)

};

FXDEFMAP(GNEGenericDataFrame::IntervalSelector) IntervalSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CREATE,                  GNEGenericDataFrame::IntervalSelector::onCmdCreateInterval),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAINTERVAL_SELECTED,   GNEGenericDataFrame::IntervalSelector::onCmdSelectInterval),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SET_ATTRIBUTE,           GNEGenericDataFrame::IntervalSelector::onCmdSetIntervalAttribute),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SELECT,                  GNEGenericDataFrame::IntervalSelector::onCmdSelectCheckButton)
};

FXDEFMAP(GNEGenericDataFrame::AttributeSelector) AttributeSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SELECT,  GNEGenericDataFrame::AttributeSelector::onCmdSelectAttribute)
};

// Object implementation
FXIMPLEMENT(GNEGenericDataFrame::DataSetSelector,   MFXGroupBoxModule, DataSetSelectorMap,   ARRAYNUMBER(DataSetSelectorMap))
FXIMPLEMENT(GNEGenericDataFrame::IntervalSelector,  MFXGroupBoxModule, IntervalSelectorMap,  ARRAYNUMBER(IntervalSelectorMap))
FXIMPLEMENT(GNEGenericDataFrame::AttributeSelector, MFXGroupBoxModule, AttributeSelectorMap, ARRAYNUMBER(AttributeSelectorMap))

// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNEGenericDataFrame::DataSetSelector - methods
// ---------------------------------------------------------------------------

GNEGenericDataFrame::DataSetSelector::DataSetSelector(GNEGenericDataFrame* genericDataFrameParent) :
    MFXGroupBoxModule(genericDataFrameParent, TL("DataSet")),
    myGenericDataFrameParent(genericDataFrameParent) {
    // create check button for new data set
    myNewDataSetCheckButton = new FXCheckButton(getCollapsableFrame(), TL("Create new dataSet"), this, MID_GNE_SELECT, GUIDesignCheckButton);
    // Create MFXComboBoxIcon
    myDataSetsComboBox = new MFXComboBoxIcon(getCollapsableFrame(), GUIDesignComboBoxNCol, true, GUIDesignComboBoxVisibleItemsMedium,
            this, MID_GNE_DATASET_SELECTED, GUIDesignComboBox);
    // create new id label
    myHorizontalFrameNewID = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(myHorizontalFrameNewID, "new dataSet ID", nullptr, GUIDesignLabelThickedFixed(100));
    // create new id textField
    myNewDataSetIDTextField = new FXTextField(myHorizontalFrameNewID, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    // hide horizontal frame
    myHorizontalFrameNewID->hide();
    // create dataSet button
    myCreateDataSetButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Create dataSet"), "", "", GUIIconSubSys::getIcon(GUIIcon::DATASET), this, MID_GNE_CREATE, GUIDesignButton);
    myCreateDataSetButton->hide();
    // refresh interval selector
    refreshDataSetSelector(nullptr);
    // DataSetSelector is always shown
    show();
}


GNEGenericDataFrame::DataSetSelector::~DataSetSelector() {}


void
GNEGenericDataFrame::DataSetSelector::refreshDataSetSelector(const GNEDataSet* currentDataSet) {
    // clear items
    myDataSetsComboBox->clearItems();
    // declare item index
    int currentItemIndex = -1;
    // fill myDataSetsComboBox with all DataSets
    for (const auto& dataSet : myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getDataSets()) {
        // check if we have to set currentItemIndex
        if ((currentItemIndex == -1) && (dataSet.second == currentDataSet)) {
            currentItemIndex = myDataSetsComboBox->getNumItems();
        }
        myDataSetsComboBox->appendIconItem(dataSet.second->getID().c_str(), dataSet.second->getACIcon());
    }
    // check if we have to set current element
    if (currentItemIndex != -1) {
        myDataSetsComboBox->setCurrentItem(currentItemIndex, FALSE);
        if (myGenericDataFrameParent->getIntervalSelector()) {
            myGenericDataFrameParent->getIntervalSelector()->enableContents();
        }
    } else if (myGenericDataFrameParent->getIntervalSelector()) {
        myGenericDataFrameParent->getIntervalSelector()->disableContents();
    }
    // recalc frame
    recalc();
    // refresh interval selector
    if (myGenericDataFrameParent->myIntervalSelector) {
        myGenericDataFrameParent->myIntervalSelector->refreshIntervalSelector();
    }
}


GNEDataSet*
GNEGenericDataFrame::DataSetSelector::getDataSet() const {
    if ((myNewDataSetCheckButton->getCheck() == TRUE) || (myDataSetsComboBox->getNumItems() == 0)) {
        return nullptr;
    } else {
        return myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveDataSet(myDataSetsComboBox->getItemText(myDataSetsComboBox->getCurrentItem()), false);
    }
}


long
GNEGenericDataFrame::DataSetSelector::onCmdCreateDataSet(FXObject*, FXSelector, void*) {
    // first disable interval selector
    myGenericDataFrameParent->getIntervalSelector()->disableContents();
    // get string
    const std::string dataSetID = myNewDataSetIDTextField->getText().text();
    // check conditions
    if (myNewDataSetIDTextField->getTextColor() == FXRGB(255, 0, 0)) {
        WRITE_WARNING(TL("Invalid dataSet ID"));
    } else if (dataSetID.empty()) {
        WRITE_WARNING(TL("Invalid empty dataSet ID"));
    } else if (myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveDataSet(dataSetID, false) != nullptr) {
        WRITE_WARNING(TL("Invalid duplicated dataSet ID"));
    } else {
        // build data set
        GNEDataHandler dataHandler(myGenericDataFrameParent->getViewNet()->getNet(), "", true, false);
        dataHandler.buildDataSet(dataSetID);
        // refresh tag selector
        refreshDataSetSelector(myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveDataSet(dataSetID));
        // change check button
        myNewDataSetCheckButton->setCheck(FALSE, TRUE);
    }
    return 1;
}


long
GNEGenericDataFrame::DataSetSelector::onCmdSetNewDataSetID(FXObject*, FXSelector, void*) {
    //
    return 1;
}


long
GNEGenericDataFrame::DataSetSelector::onCmdSelectDataSet(FXObject*, FXSelector, void*) {
    // update interval modul
    myGenericDataFrameParent->myIntervalSelector->refreshIntervalSelector();
    return 1;
}


long
GNEGenericDataFrame::DataSetSelector::onCmdSelectCheckButton(FXObject*, FXSelector, void*) {
    if (myNewDataSetCheckButton->getCheck() == TRUE) {
        // enable textfield and label
        myHorizontalFrameNewID->show();
        myCreateDataSetButton->show();
        // disable comboBox
        myDataSetsComboBox->hide();
    } else {
        // disable textfield and label
        myHorizontalFrameNewID->hide();
        myCreateDataSetButton->hide();
        // enable comboBox
        myDataSetsComboBox->show();
    }
    // update interval modul
    myGenericDataFrameParent->myIntervalSelector->refreshIntervalSelector();
    return 1;
}

// ---------------------------------------------------------------------------
// GNEGenericDataFrame::IntervalSelector - methods
// ---------------------------------------------------------------------------

GNEGenericDataFrame::IntervalSelector::IntervalSelector(GNEGenericDataFrame* genericDataFrameParent) :
    MFXGroupBoxModule(genericDataFrameParent, TL("Interval")),
    myGenericDataFrameParent(genericDataFrameParent) {
    // create check button for new interval
    myNewIntervalCheckButton = new FXCheckButton(getCollapsableFrame(), TL("Create new interval"), this, MID_GNE_SELECT, GUIDesignCheckButton);
    // create begin label
    myHorizontalFrameBegin = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(myHorizontalFrameBegin, toString(SUMO_ATTR_BEGIN).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    // create begin TextField
    myBeginTextField = new FXTextField(myHorizontalFrameBegin, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    myBeginTextField->setText("0");
    // hide horizontal frame begin
    myHorizontalFrameBegin->hide();
    // create end label
    myHorizontalFrameEnd = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(myHorizontalFrameEnd, toString(SUMO_ATTR_END).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    // create end textfield
    myEndTextField = new FXTextField(myHorizontalFrameEnd, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    myEndTextField->setText("3600");
    // hide horizontal frame end
    myHorizontalFrameEnd->hide();
    // create interval button
    myCreateIntervalButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("create interval"), "", "", GUIIconSubSys::getIcon(GUIIcon::DATAINTERVAL), this, MID_GNE_CREATE, GUIDesignButton);
    myCreateIntervalButton->hide();
    // Create tree list with fixed height
    myIntervalsTreelist = new FXTreeList(getCollapsableFrame(), this, MID_GNE_DATAINTERVAL_SELECTED, GUIDesignTreeListFixedHeight);
    myIntervalsTreelist->setHeight(200);
    // refresh interval selector
    refreshIntervalSelector();
    // IntervalSelector is always shown
    show();
}


GNEGenericDataFrame::IntervalSelector::~IntervalSelector() {}


void
GNEGenericDataFrame::IntervalSelector::enableContents() const {
    myNewIntervalCheckButton->enable();
    myHorizontalFrameBegin->enable();
    myBeginTextField->enable();
    myHorizontalFrameEnd->enable();
    myEndTextField->enable();
    myCreateIntervalButton->enable();
    myIntervalsTreelist->enable();
}


void
GNEGenericDataFrame::IntervalSelector::disableContents() const {
    myNewIntervalCheckButton->disable();
    myHorizontalFrameBegin->disable();
    myBeginTextField->disable();
    myHorizontalFrameEnd->disable();
    myEndTextField->disable();
    myCreateIntervalButton->disable();
    myIntervalsTreelist->disable();
}


void
GNEGenericDataFrame::IntervalSelector::refreshIntervalSelector() {
    // first clear items from tree and intervalMap
    myIntervalsTreelist->clearItems();
    myTreeItemIntervalMap.clear();
    // obtain data set
    const GNEDataSet* dataSet = myGenericDataFrameParent->myDataSetSelector->getDataSet();
    // add intervals
    if (dataSet) {
        // insert dataSetItem in Tree list
        FXTreeItem* dataSetItem = myIntervalsTreelist->insertItem(
                                      nullptr, nullptr,
                                      dataSet->getHierarchyName().c_str(),
                                      GUIIconSubSys::getIcon(GUIIcon::DATASET),
                                      GUIIconSubSys::getIcon(GUIIcon::DATASET));
        // by default item is expanded
        dataSetItem->setExpanded(true);
        // iterate over intevals
        for (const auto& interval : dataSet->getDataIntervalChildren()) {
            addIntervalItem(interval.second, dataSetItem);
        }
    }
    // refresh attribute selector
    if (myGenericDataFrameParent->myAttributeSelector) {
        myGenericDataFrameParent->myAttributeSelector->refreshAttributeSelector();
    }
    // recalc frame
    recalc();
}


GNEDataInterval*
GNEGenericDataFrame::IntervalSelector::getDataInterval() const {
    // first check if there is elements in interval tree
    if (myIntervalsTreelist->getNumItems() > 0) {
        for (const auto& treeItem : myTreeItemIntervalMap) {
            if (treeItem.first->isSelected()) {
                return treeItem.second;
            }
        }
    }
    // no GNEDataInterval found, then return nullptr
    return nullptr;
}


long
GNEGenericDataFrame::IntervalSelector::onCmdCreateInterval(FXObject*, FXSelector, void*) {
    // first check that begin and end are valid
    if (GNEAttributeCarrier::canParse<double>(myBeginTextField->getText().text()) &&
            GNEAttributeCarrier::canParse<double>(myEndTextField->getText().text())) {
        // obtain begin and end
        const double begin = GNEAttributeCarrier::parse<double>(myBeginTextField->getText().text());
        const double end = GNEAttributeCarrier::parse<double>(myEndTextField->getText().text());
        // get data set parent
        GNEDataSet* dataSet = myGenericDataFrameParent->myDataSetSelector->getDataSet();
        if (dataSet && dataSet->checkNewInterval(begin, end)) {
            // declare dataHandler
            GNEDataHandler dataHandler(myGenericDataFrameParent->getViewNet()->getNet(), "", true, false);
            // build data interval
            dataHandler.buildDataInterval(nullptr, dataSet->getID(), begin, end);
        }
        // disable select interval check button
        myNewIntervalCheckButton->setCheck(FALSE, TRUE);
    }
    return 1;
}


long
GNEGenericDataFrame::IntervalSelector::onCmdSelectInterval(FXObject*, FXSelector, void*) {
    // refresh attribute selector
    myGenericDataFrameParent->myAttributeSelector->refreshAttributeSelector();
    return 1;
}


long
GNEGenericDataFrame::IntervalSelector::onCmdSetIntervalAttribute(FXObject* obj, FXSelector, void*) {
    if (obj == myBeginTextField) {
        // check if begin value can be parsed to double
        if (GNEAttributeCarrier::canParse<double>(myBeginTextField->getText().text())) {
            myBeginTextField->setTextColor(FXRGB(0, 0, 0));
            myBeginTextField->killFocus();
        } else {
            myBeginTextField->setTextColor(FXRGB(255, 0, 0));
        }
    } else if (obj == myEndTextField) {
        // check if end value can be parsed to double
        if (GNEAttributeCarrier::canParse<double>(myEndTextField->getText().text())) {
            myEndTextField->setTextColor(FXRGB(0, 0, 0));
            myEndTextField->killFocus();
        } else {
            myEndTextField->setTextColor(FXRGB(255, 0, 0));
        }
    }
    return 1;
}


long
GNEGenericDataFrame::IntervalSelector::onCmdSelectCheckButton(FXObject*, FXSelector, void*) {
    if (myNewIntervalCheckButton->getCheck() == TRUE) {
        // enable begin and end elements
        myHorizontalFrameBegin->show();
        myHorizontalFrameEnd->show();
        myCreateIntervalButton->show();
        // refresh begin and end text fields
        const GNEDataSet* dataSet = myGenericDataFrameParent->myDataSetSelector->getDataSet();
        if (dataSet) {
            if (dataSet->getDataIntervalChildren().empty()) {
                // set default interval (1 hour)
                myBeginTextField->setText("0");
                myEndTextField->setText("3600");
            } else {
                // obtain last data interval
                const GNEDataInterval* lastDataInterval = dataSet->getDataIntervalChildren().rbegin()->second;
                const double intervalDuration = lastDataInterval->getAttributeDouble(SUMO_ATTR_END) - lastDataInterval->getAttributeDouble(SUMO_ATTR_BEGIN);
                // set new begin end
                myBeginTextField->setText(toString(lastDataInterval->getAttributeDouble(SUMO_ATTR_END)).c_str());
                myEndTextField->setText(toString(lastDataInterval->getAttributeDouble(SUMO_ATTR_END) + intervalDuration).c_str());
            }
        }
    } else {
        // disable begin and end elements
        myHorizontalFrameBegin->hide();
        myHorizontalFrameEnd->hide();
        myCreateIntervalButton->hide();
    }
    // refresh interval seletor
    refreshIntervalSelector();
    return 1;
}


FXTreeItem*
GNEGenericDataFrame::IntervalSelector::addIntervalItem(GNEDataInterval* dataInterval, FXTreeItem* itemParent) {
    // insert item in Tree list
    FXTreeItem* item = myIntervalsTreelist->insertItem(nullptr, itemParent,
                       dataInterval->getHierarchyName().c_str(),
                       GUIIconSubSys::getIcon(GUIIcon::DATAINTERVAL),
                       GUIIconSubSys::getIcon(GUIIcon::DATAINTERVAL));
    // insert item in map
    myTreeItemIntervalMap[item] = dataInterval;
    // by default item is expanded
    item->setExpanded(true);
    // select first item
    if (myTreeItemIntervalMap.size() == 1) {
        item->setSelected(TRUE);
    }
    // return created FXTreeItem
    return item;
}

// ---------------------------------------------------------------------------
// GNEGenericDataFrame::AttributeSelector - methods
// ---------------------------------------------------------------------------

GNEGenericDataFrame::AttributeSelector::AttributeSelector(GNEGenericDataFrame* genericDataFrameParent, SumoXMLTag tag) :
    MFXGroupBoxModule(genericDataFrameParent, TL("Data attributes")),
    myGenericDataFrameParent(genericDataFrameParent),
    myMinMaxLabel(nullptr),
    myGenericDataTag(tag) {
    // Create MFXComboBoxIcon
    myAttributesComboBox = new MFXComboBoxIcon(getCollapsableFrame(), GUIDesignComboBoxNCol, true, GUIDesignComboBoxVisibleItemsMedium,
            this, MID_GNE_SELECT, GUIDesignComboBox);
    // build rainbow
    myMinMaxLabel = buildRainbow(this);
    // refresh interval selector
    refreshAttributeSelector();
    // AttributeSelector is always shown
    show();
}


GNEGenericDataFrame::AttributeSelector::~AttributeSelector() {}


void
GNEGenericDataFrame::AttributeSelector::refreshAttributeSelector() {
    // save current attribute
    const auto currentAttribute = myAttributesComboBox->getText();
    // clear items
    myAttributesComboBox->clearItems();
    // restore myMinMaxLabel
    myMinMaxLabel->setText(TL("Scale: Min -> Max"));
    // fill myAttributesComboBox depending of data sets
    if (myGenericDataFrameParent->myDataSetSelector->getDataSet() == nullptr) {
        myAttributesComboBox->appendIconItem("<no dataSet selected>");
        myAttributesComboBox->disable();
    } else {
        // add all item
        myAttributesComboBox->appendIconItem("<all>");
        // add attributes depending of interval
        if (myGenericDataFrameParent->myIntervalSelector->getDataInterval() == nullptr) {
            const auto parameters = myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveGenericDataParameters(
                                        myGenericDataFrameParent->myDataSetSelector->getDataSet()->getID(), toString(myGenericDataTag), "", "");
            // add all parameters
            for (const auto& attribute : parameters) {
                myAttributesComboBox->appendIconItem(attribute.c_str());
            }
        } else {
            // retrieve all parameters within begin and end
            const auto parameters = myGenericDataFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveGenericDataParameters(
                                        myGenericDataFrameParent->myDataSetSelector->getDataSet()->getID(), toString(myGenericDataTag),
                                        myGenericDataFrameParent->myIntervalSelector->getDataInterval()->getAttribute(SUMO_ATTR_BEGIN),
                                        myGenericDataFrameParent->myIntervalSelector->getDataInterval()->getAttribute(SUMO_ATTR_END));
            // add all parameters
            for (const auto& attribute : parameters) {
                myAttributesComboBox->appendIconItem(attribute.c_str());
            }
        }
        // enable combo Box
        myAttributesComboBox->enable();
        // set current item
        for (int i = 0; i < myAttributesComboBox->getNumItems(); i++) {
            if (myAttributesComboBox->getItemText(i) == currentAttribute.text()) {
                myAttributesComboBox->setCurrentItem(i, TRUE);
            }
        }
    }
    // recalc frame
    recalc();
    // update view net
    myGenericDataFrameParent->getViewNet()->updateViewNet();
}


std::string
GNEGenericDataFrame::AttributeSelector::getFilteredAttribute() const {
    if (myAttributesComboBox->getNumItems() == 0) {
        return "";
    } else if (myAttributesComboBox->getText() == TL("<all>")) {
        return "";
    } else {
        return myAttributesComboBox->getText().text();
    }
}


const RGBColor&
GNEGenericDataFrame::AttributeSelector::getScaledColor(const double min, const double max, const double value) const {
    // update myMinMaxLabel
    myMinMaxLabel->setText(("Min: " + toString(min) + " -> Max: " + toString(max)).c_str());
    // return scaled color
    return GNEViewNetHelper::getRainbowScaledColor(min, max, value);
}


long
GNEGenericDataFrame::AttributeSelector::onCmdSelectAttribute(FXObject*, FXSelector, void*) {
    // empty attribute means <all>
    if (myAttributesComboBox->getText().empty()) {
        myAttributesComboBox->setCurrentItem(0);
    }
    if (myAttributesComboBox->getText() == TL("<all>")) {
        myMinMaxLabel->setText(TL("Scale: Min -> Max"));
    }
    // update view
    myGenericDataFrameParent->getViewNet()->updateViewNet();
    return 1;
}

// ---------------------------------------------------------------------------
// GNEGenericDataFrame - methods
// ---------------------------------------------------------------------------

const GNEGenericDataFrame::DataSetSelector*
GNEGenericDataFrame::getDataSetSelector() const {
    return myDataSetSelector;
}


const GNEGenericDataFrame::IntervalSelector*
GNEGenericDataFrame::getIntervalSelector() const {
    return myIntervalSelector;
}


const GNEGenericDataFrame::AttributeSelector*
GNEGenericDataFrame::getAttributeSelector() const {
    return myAttributeSelector;
}


GNEPathCreator*
GNEGenericDataFrame::getPathCreator() const {
    return myPathCreator;
}


SumoXMLTag
GNEGenericDataFrame::getTag() const {
    return myGenericDataTag;
}


void
GNEGenericDataFrame::show() {
    // first refresh data set selector
    myDataSetSelector->refreshDataSetSelector(nullptr);
    // check if there is an edge path creator
    if (myPathCreator) {
        myPathCreator->showPathCreatorModule(GNEAttributeCarrier::getTagProperty(myGenericDataTag), false);
    }
    // show frame
    GNEFrame::show();
}


void
GNEGenericDataFrame::hide() {
    if (myPathCreator) {
        // reset candidate edges
        for (const auto& edge : myViewNet->getNet()->getAttributeCarriers()->getEdges()) {
            edge.second->resetCandidateFlags();
        }
    }
    // hide frame
    GNEFrame::hide();
}


void
GNEGenericDataFrame::updateFrameAfterUndoRedo() {
    // refresh data set selector
    myDataSetSelector->refreshDataSetSelector(nullptr);
    // check if there is an edge path creator
    if (myPathCreator) {
        myPathCreator->showPathCreatorModule(GNEAttributeCarrier::getTagProperty(myGenericDataTag), false);
    }
}


GNEGenericDataFrame::GNEGenericDataFrame(GNEViewParent* viewParent, GNEViewNet* viewNet, SumoXMLTag tag, const bool pathCreator) :
    GNEFrame(viewParent, viewNet, toString(tag)),
    myDataSetSelector(nullptr),
    myIntervalSelector(nullptr),
    myAttributeSelector(nullptr),
    myGenericDataAttributes(nullptr),
    myPathCreator(nullptr),
    myGenericDataTag(tag) {
    // create DataSetSelector
    myDataSetSelector = new DataSetSelector(this);
    // create IntervalSelector module
    myIntervalSelector = new IntervalSelector(this);
    // create AttributeSelector module
    myAttributeSelector = new AttributeSelector(this, tag);
    // create parameter editor module
    myGenericDataAttributes = new GNEFrameAttributeModules::GenericDataAttributes(this);
    // create GNEPathCreator module
    if (pathCreator) {
        myPathCreator = new GNEPathCreator(this);
    }
}


GNEGenericDataFrame::~GNEGenericDataFrame() {}


void
GNEGenericDataFrame::intervalSelected() {
    //
}


bool
GNEGenericDataFrame::createPath(const bool /*useLastRoute*/) {
    // this function has to be reimplemented in all child frames that uses a GNEPathCreator
    return false;
}

/****************************************************************************/
