#include "PCLPlugin.h"
#include "PointSetToMeshConversionDialog.h"
#include <cnoid/MainMenu>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/PointSetItem>

#include <cnoid/SceneDrawables>
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>

#include <pcl/io/pcd_io.h>

#include <iostream>

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

PCLPlugin* instance_ = nullptr;

class PCLPcdFileIo : public ItemFileIoBase<PointSetItem>
{
public:
    PCLPcdFileIo();
    virtual bool load(PointSetItem* item, const std::string& filename) override;
};

PCLPcdFileIo::PCLPcdFileIo()
    : ItemFileIoBase("PCL-PCD", Load)
{
    setCaption(_("PCL_pcd"));
    setFileTypeCaption("PCD");
    setExtensionForLoading("pcd");
    addFormatAlias("PCL-PCD");
}

bool PCLPcdFileIo::load(PointSetItem* item, const std::string& filename)
{
    pcl::PCDReader rd;
    //
    //pcl::PCLPointCloud2 hdr;
    //rd.readHeader(filename, hdr);
    // TODO: checking colors
    pcl::PointCloud<pcl::PointXYZ> pt;
    rd.read<pcl::PointXYZ>(filename, pt);
#if 0
    Vector3 ave(0.0, 0.0, 0.0);
    for (auto it = pt.begin(); it != pt.end(); it++) {
        ave.x() += it->x;
        ave.y() += it->y;
        ave.z() += it->z;
    }
    std::cerr << "width: " << pt.width << std::endl;
    std::cerr << "height: " << pt.height << std::endl;
    ave /= (pt.width*pt.height);
    std::cerr << "ave: " << ave.x() << ", " << ave.y() << ", " << ave.z() << std::endl;

    SgVertexArray *vt = item->pointSet()->getOrCreateVertices();

    int cntr = 0;
    for (auto it = pt.begin(); it != pt.end(); it++) {
        Vector3f v(it->x, it->y, it->z);
        v -= avef;
        v /= 128;
        vt->push_back(v);
    }
#endif
    SgVertexArray *vt = item->pointSet()->getOrCreateVertices();
    int cntr = 0;
    for (auto it = pt.begin(); it != pt.end(); it++) {
        Vector3f v(it->x, it->y, it->z);
        vt->push_back(v);
    }

    // std::cerr << item->pointSet()->vertices()->size() << " points have been loaded." << std::endl;

    auto itype = currentInvocationType();
    if(itype == Dialog || itype == DragAndDrop){
        item->setChecked(true);
    }

    return true;
}

}


PCLPlugin::PCLPlugin()
    : Plugin("PCL")
{
    require("Body");    
    instance_ = this;
}


PCLPlugin* PCLPlugin::instance()
{
    return instance_;
}


bool PCLPlugin::initialize()
{
    PointSetToMeshConversionDialog::initializeClass(this);
    
    auto mainMenu = MainMenu::instance();

    mainMenu->add_Filters_Item(
        _("Convert point set to mesh"),
        [this](){ PointSetToMeshConversionDialog::instance()->show(); });

    ItemTreeView::customizeContextMenu<PointSetItem>(
        [this](PointSetItem* item, MenuManager& menu, ItemFunctionDispatcher menuFunction){
            menu.addItem(_("Convert to mesh"))->sigTriggered().connect(
                [this, item](){ PointSetToMeshConversionDialog::instance()->show(item); });
            menu.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
    
    ItemManager& im = this->itemManager();
    im.registerClass<PointSetItem>(N_("PointSetItem"));
    im.addFileIO<PointSetItem>(new PCLPcdFileIo);
    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(PCLPlugin);
