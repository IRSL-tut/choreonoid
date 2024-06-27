#include "FileBar.h"
#include "ExtensionManager.h"
#include "ProjectManager.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;


void FileBar::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->addToolBar(instance());
        initialized = true;
    }
}


FileBar* FileBar::instance()
{
    static FileBar* fileBar = new FileBar;
    return fileBar;
}


FileBar::FileBar()
    : ToolBar(N_("FileBar"))
{
    setVisibleByDefault(true);

    auto button = addButton(":/Base/icon/projectsave.svg");
    button->setToolTip(_("Save the project"));
    button->sigClicked().connect([](){ ProjectManager::instance()->overwriteCurrentProject(); });
}


FileBar::~FileBar()
{

}
