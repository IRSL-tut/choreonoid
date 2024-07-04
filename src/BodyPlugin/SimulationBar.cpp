#include "SimulationBar.h"
#include "SimulatorItem.h"
#include "WorldItem.h"
#include <cnoid/ExtensionManager>
#include <cnoid/OptionManager>
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/UnifiedEditHistory>
#include <cnoid/Archive>
#include <cnoid/QtEventUtil>
#include <cnoid/Format>
#include <functional>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

SimulationBar* instance_ = nullptr;
bool doStartSimulation = false;

void onSigOptionsParsed(OptionManager*)
{
    if(doStartSimulation){
        instance_->startSimulation(true);
    }
}

}


void SimulationBar::initialize(ExtensionManager* ext)
{
    if(!instance_){
        instance_ = new SimulationBar;
        ext->addToolBar(instance_);

        auto om = OptionManager::instance();
        om->add_flag("--start-simulation", doStartSimulation, "start simulation automatically");
        om->sigOptionsParsed(1).connect(onSigOptionsParsed);
    }
}


SimulationBar* SimulationBar::instance()
{
    return instance_;
}


SimulationBar::SimulationBar()
    : ToolBar(N_("SimulationBar"))
{
    auto storeButton = addButton(":/Body/icon/store-world-initial.svg");
    storeButton->setToolTip(_("Store body positions to the initial world state"));
    storeButton->sigClicked().connect([this](){ onStoreInitialClicked(); });
    
    auto restoreButton = addButton(":/Body/icon/restore-world-initial.svg");
    restoreButton->setToolTip(_("Restore body positions from the initial world state"));
    restoreButton->sigClicked().connect([this](){ onRestoreInitialClicked(); });

    auto startButton = addButton(":/Body/icon/start-simulation.svg");
    startButton->setToolTip(_("Start simulation from the beginning"));
    startButton->sigClicked().connect([this](){ startSimulation(true); });

    auto restartButton = addButton(":/Body/icon/restart-simulation.svg");
    restartButton->setToolTip(_("Start simulation from the current state"));
    restartButton->sigClicked().connect([this](){ startSimulation(false); });
    
    pauseToggle = addToggleButton(":/Body/icon/pause-simulation.svg");
    pauseToggle->setToolTip(_("Pause simulation"));
    pauseToggle->sigClicked().connect([this](){ onPauseSimulationClicked(); });
    pauseToggle->setChecked(false);

    stopButton = addButton(":/Body/icon/stop-simulation.svg");
    stopButton->setToolTip(_("Stop simulation"));
    stopButton->sigClicked().connect([this](){ onStopSimulationClicked(); });
    stopButton->installEventFilter(this);
    isStopConfirmationEnabled = false;
}


SimulationBar::~SimulationBar()
{

}


static void forEachTargetBodyItem(std::function<void(BodyItem*)> callback)
{
    for(auto& bodyItem : RootItem::instance()->descendantItems<BodyItem>()){
        bool isTarget = bodyItem->isSelected();
        if(!isTarget){
            WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();
            if(worldItem && worldItem->isSelected()){
                isTarget = true;
            }
        }
        if(isTarget){
            callback(bodyItem);
        }
    }
}


static void storeInitialBodyState(BodyItem* bodyItem)
{
    bodyItem->storeInitialState();
    MessageView::instance()->putln(
        formatR(_("Current state of {} has been set to the initial state."), bodyItem->displayName()));
}


void SimulationBar::onStoreInitialClicked()
{
    forEachTargetBodyItem(storeInitialBodyState);
}


void SimulationBar::onRestoreInitialClicked()
{
    auto history = UnifiedEditHistory::instance();
    history->beginEditGroup(_("Restore body initial positions"));
    
    forEachTargetBodyItem(
        [](BodyItem* bodyItem){ bodyItem->restoreInitialState(true); });
    
    history->endEditGroup();
}


void SimulationBar::forEachSimulator(std::function<void(SimulatorItem* simulator)> callback, bool doSelect)
{
    auto mv = MessageView::instance();

    auto rootItem = RootItem::instance();
    auto simulators =  rootItem->selectedItems<SimulatorItem>();
    if(simulators.empty()){
        if(auto simulator = rootItem->findItem<SimulatorItem>()){
            if(doSelect){
                simulator->setSelected(true);
            }
            simulators.push_back(simulator);
        } else {
            mv->notify(_("There is no simulator item."));
        }
    }

    map<WorldItem*, SimulatorItem*> worldToSimulatorMap;

    for(auto& simulator : simulators){
        if(auto world = simulator->findOwnerItem<WorldItem>()){
            auto p = worldToSimulatorMap.find(world);
            if(p == worldToSimulatorMap.end()){
                worldToSimulatorMap[world] = simulator;
            } else {
                p->second = nullptr; // skip if multiple simulators are selected
            }
        }
    }

    for(auto& simulator : simulators){
        auto world = simulator->findOwnerItem<WorldItem>();
        if(!world){
            mv->notify(formatR(_("{} cannot be processed because it is not related with a world."),
                               simulator->displayName()));
        } else {
            if(auto simulator2 = worldToSimulatorMap[world]){
                callback(simulator);
            } else {
                mv->notify(
                    formatR(_("{} cannot be processed because another simulator in the same world is also selected."),
                            simulator->displayName()),
                    MessageView::Warning);
            }
        }
    }
}


void SimulationBar::startSimulation(bool doReset)
{
    forEachSimulator(
        [=](SimulatorItem* simulator){ startSimulation(simulator, doReset); },
        true);
}


void SimulationBar::startSimulation(SimulatorItem* simulator, bool doReset)
{
    if(simulator->isRunning()){
    	if(pauseToggle->isChecked() && !doReset){
            simulator->restartSimulation();
            pauseToggle->blockSignals(true);
            pauseToggle->setChecked(false);
            pauseToggle->blockSignals(false);
    	}
        TimeBar::instance()->startPlayback();
        
    } else {
        sigSimulationAboutToStart_(simulator);
        simulator->startSimulation(doReset);
        pauseToggle->blockSignals(true);
        pauseToggle->setChecked(false);
        pauseToggle->blockSignals(false);
    }
}


bool SimulationBar::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == stopButton && event->type() == QEvent::MouseButtonPress){
        auto mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == Qt::RightButton){
            onStopButtonRightClicked(mouseEvent);
            return true;
        }
    }
    // standard event processing
    return QObject::eventFilter(obj, event);
}


void SimulationBar::onStopButtonRightClicked(QMouseEvent* event)
{
    menuManager.setNewPopupMenu(stopButton);
    auto check = menuManager.addCheckItem(_("Enable Confirmation Dialog"));
    check->setChecked(isStopConfirmationEnabled);
    check->sigToggled().connect([this](bool on){ isStopConfirmationEnabled = on; });
    menuManager.popupMenu()->popup(getGlobalPosition(event));
}


void SimulationBar::onStopSimulationClicked()
{
    if(isStopConfirmationEnabled){
        if(!showConfirmDialog(
               _("Stop Simulation"),
               _("Do you really want to stop the simulation completely?"))){
            return;
        }
    }
    
    pauseToggle->blockSignals(true);
    pauseToggle->setChecked(false);
    pauseToggle->blockSignals(false);

    forEachSimulator(
        [](SimulatorItem* simulator){ simulator->stopSimulation(true); });
}


void SimulationBar::onPauseSimulationClicked()
{
    forEachSimulator(
        [this](SimulatorItem* simulator){ pauseSimulation(simulator); });
}


void SimulationBar::pauseSimulation(SimulatorItem* simulator)
{
    auto timeBar = TimeBar::instance();
    
    if(pauseToggle->isChecked()){
        if(simulator->isRunning()){
            simulator->pauseSimulation();
        }
        if(timeBar->isDoingPlayback()){
            timeBar->stopPlayback();
        }
    } else {
        if(simulator->isRunning()){
            simulator->restartSimulation();
        }
        timeBar->startPlayback();
    }
}


bool SimulationBar::storeState(Archive& archive)
{
    if(isStopConfirmationEnabled){
        archive.write("is_stop_confirmation_enabled", true);
    }
    return true;
}


bool SimulationBar::restoreState(const Archive& archive)
{
    archive.read("is_stop_confirmation_enabled", isStopConfirmationEnabled);
    return true;
}
