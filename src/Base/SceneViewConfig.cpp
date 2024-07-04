#include "SceneViewConfig.h"
#include "SceneView.h"
#include "RootItem.h"
#include "Archive.h"
#include "Dialog.h"
#include "Separator.h"
#include "CheckBox.h"
#include <QBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDialogButtonBox>
#include <cnoid/Format>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class SceneViewConfig::Impl
{
public:
    SceneViewConfig* self;
    vector<SceneView*> sceneViews;
    CheckBox* dedicatedItemCheckCheck;
    int itemCheckId;
    QWidget* viewOptionPanel;
    Dialog* dialog;

    Impl(SceneViewConfig* self);
    Impl(const Impl& org, SceneViewConfig* self);
    ~Impl();
    void doCommonInitialization();
    void onDedicatedItemCheckToggled(bool on);    
    void updateSceneView(SceneView* view);
    void updateSceneViews();
    bool restore(const Archive* archive);
    QWidget* getOrCreateViewOptionPanel();
    void createConfigDialog();    
};

}


SceneViewConfig::SceneViewConfig()
{
    impl = new Impl(this);
}


SceneViewConfig::Impl::Impl(SceneViewConfig* self)
    : self(self)
{
    itemCheckId = Item::PrimaryCheck;

    doCommonInitialization();
}


SceneViewConfig::SceneViewConfig(const SceneViewConfig& org)
    : SceneWidgetConfig(org)
{
    impl = new Impl(*org.impl, this);
}


SceneViewConfig::Impl::Impl(const Impl& org, SceneViewConfig* self)
    : self(self)
{
    itemCheckId = org.itemCheckId;
    doCommonInitialization();
}


void SceneViewConfig::Impl::doCommonInitialization()
{
    dedicatedItemCheckCheck =
        new CheckBox(_("Use dedicated item tree view checks to select the target items"));
    dedicatedItemCheckCheck->setChecked(itemCheckId != Item::PrimaryCheck);
    dedicatedItemCheckCheck->sigToggled().connect(
        [&](bool on){ onDedicatedItemCheckToggled(on); });
    
    viewOptionPanel = nullptr;
    dialog = nullptr;
}


SceneViewConfig::~SceneViewConfig()
{
    delete impl;
}


SceneViewConfig::Impl::~Impl()
{
    if(itemCheckId != Item::PrimaryCheck){
        RootItem::instance()->releaseCheckEntry(itemCheckId);
    }
    delete dedicatedItemCheckCheck;
    
    if(viewOptionPanel){
        delete viewOptionPanel;
    }
    if(dialog){
        delete dialog;
    }
}


void SceneViewConfig::addSceneView(SceneView* view, bool doUpdateSceneView)
{
    impl->sceneViews.push_back(view);
    addSceneWidget(view->sceneWidget(), doUpdateSceneView);

    if(doUpdateSceneView){
        impl->updateSceneView(view);
    }
}


void SceneViewConfig::removeSceneView(SceneView* view)
{
    removeSceneWidget(view->sceneWidget());
    
    auto& views = impl->sceneViews;
    auto it = std::find(views.begin(), views.end(), view);
    if(it != views.end()){
        views.erase(it);
    }
}


void SceneViewConfig::clearSceneViews()
{
    impl->sceneViews.clear();
    clearSceneWidgets();
}


string SceneViewConfig::getTargetSceneViewSetCaption()
{
    string caption;
    if(impl->sceneViews.empty()){
        caption = _("Scene view");
    } else {
        for(auto& view : impl->sceneViews){
            if(!caption.empty()){
                caption += ", ";
            }
            caption += view->windowTitle().toStdString();
        }
    }
    return caption;
}


void SceneViewConfig::Impl::onDedicatedItemCheckToggled(bool on)
{
    int checkId = Item::PrimaryCheck;

    if(on){
        if(itemCheckId == Item::PrimaryCheck){
            itemCheckId =
                RootItem::instance()->addCheckEntry(self->getTargetSceneViewSetCaption());
        }
        checkId = itemCheckId;

    } else {
        if(itemCheckId != Item::PrimaryCheck){
            RootItem::instance()->releaseCheckEntry(itemCheckId);
            itemCheckId = Item::PrimaryCheck;
        }
    }

    updateSceneViews();
}


void SceneViewConfig::Impl::updateSceneView(SceneView* view)
{
    view->setTargetSceneItemCheckId(itemCheckId);
}


void SceneViewConfig::Impl::updateSceneViews()
{
    for(auto& view : sceneViews){
        updateSceneView(view);
    }
}


// Public version of this function updates all the targets including
// those implemented in the base classes
void SceneViewConfig::updateSceneViews()
{
    updateSceneWidgets();
    impl->updateSceneViews();
}


bool SceneViewConfig::store(Archive* archive)
{
    bool result = SceneWidgetConfig::store(archive);

    if(impl->itemCheckId != Item::PrimaryCheck){
        archive->write("isDedicatedItemCheckEnabled", true);
        RootItem::instance()->storeCheckStates(impl->itemCheckId, *archive, "checked");
    }

    return result;
}


bool SceneViewConfig::restore(const Archive* archive)
{
    bool result = SceneWidgetConfig::restore(archive);
    return impl->restore(archive) && result;
}


bool SceneViewConfig::Impl::restore(const Archive* archive)
{
    bool isDedicatedItemCheckEnabled =
        archive->get("isDedicatedItemCheckEnabled", false) ||
        archive->get("dedicatedItemTreeViewChecks", false);

    // This will also call the onDedicatedCheckToggled() function, which
    // updates the itemCheckId.
    dedicatedItemCheckCheck->setChecked(isDedicatedItemCheckEnabled);
    
    if(itemCheckId != Item::PrimaryCheck){
        ref_ptr<const Archive> pArchive = archive;
        archive->addPostProcess(
            [this, pArchive](){
                RootItem::instance()->restoreCheckStates(itemCheckId, *pArchive, "checked");
            });
    }

    return true;
}


QWidget* SceneViewConfig::getOrCreateViewOptionaPanel()
{
    return impl->getOrCreateViewOptionPanel();
}


QWidget* SceneViewConfig::Impl::getOrCreateViewOptionPanel()
{
    if(!viewOptionPanel){
        viewOptionPanel = new QWidget;
        auto hbox = new QHBoxLayout;
        hbox->setContentsMargins(0, 0, 0, 0);
        hbox->addWidget(dedicatedItemCheckCheck);
        hbox->addStretch();
        viewOptionPanel->setLayout(hbox);
    }
    return viewOptionPanel;
}


void SceneViewConfig::showConfigDialog()
{
    if(!impl->dialog){
        impl->createConfigDialog();
    }
    impl->dialog->show();
}

    
void SceneViewConfig::Impl::createConfigDialog()
{
    dialog = new Dialog;
    dialog->setWindowTitle(
        formatR(_("Configuration of {0}"), self->getTargetSceneViewSetCaption()).c_str());

    auto vbox = new QVBoxLayout;
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Camera"))));
    vbox->addWidget(self->getOrCreateCameraPanel());
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Lighting"))));
    vbox->addWidget(self->getOrCreateLightingPanel());
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Background"))));
    vbox->addWidget(self->getOrCreateBackgroundPanel());
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Drawing"))));
    vbox->addWidget(self->getOrCreateDrawingPanel());

    if(false){
        vbox->addLayout(new HSeparatorBox(new QLabel(_("Debug"))));
        vbox->addWidget(self->getOrCreateDebugPanel());
    }
    
    vbox->addSpacing(4);
    vbox->addWidget(new HSeparator);
    vbox->addWidget(getOrCreateViewOptionPanel());
    vbox->addSpacing(4);
    vbox->addWidget(new HSeparator);

    auto okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    auto buttonBox = new QDialogButtonBox(dialog);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    QObject::connect(buttonBox, SIGNAL(accepted()), dialog, SLOT(accept()));
    vbox->addWidget(buttonBox);
    
    dialog->setLayout(vbox);
}
