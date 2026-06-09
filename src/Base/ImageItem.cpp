#include "ImageItem.h"
#include "ItemManager.h"
#include "ItemFileIO.h"
#include "Archive.h"
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ImageFileIO : public ItemFileIO
{
public:
    ImageFileIO()
        : ItemFileIO("IMAGE-FILE", Load)
    {
        setCaption(_("Image"));
        setFileTypeCaption(_("Image File"));
        setExtensions({ "png", "jpg", "jpeg", "tga", "tif", "tiff", "hdr" });
    }

    virtual Item* createItem() override
    {
        return new ImageItem;
    }

    virtual bool load(Item* item, const std::string& filename) override
    {
        return static_cast<ImageItem*>(item)->loadImageFile(filename, os());
    }
};

}


void ImageItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        auto& im = ext->itemManager();
        im.registerClass<ImageItem>(N_("ImageItem"));
        im.addFileIO<ImageItem>(new ImageFileIO);
        initialized = true;
    }
}


ImageItem::ImageItem()
{
    setAttributes(FileImmutable | Reloadable);
    image_ = make_shared<Image>();
}


ImageItem::ImageItem(const ImageItem& org)
    : Item(org)
{
    image_ = make_shared<Image>(*org.image_);
}


ImageItem::~ImageItem()
{

}


Item* ImageItem::doCloneItem(CloneMap*) const
{
    return new ImageItem(*this);
}


const Image* ImageItem::getImage()
{
    return (image_ && !image_->empty()) ? image_.get() : nullptr;
}


SignalProxy<void()> ImageItem::sigImageUpdated()
{
    return sigImageUpdated_;
}


void ImageItem::setImage(std::shared_ptr<Image> image)
{
    image_ = image;
    sigImageUpdated_();
}


bool ImageItem::loadImageFile(const std::string& filename, std::ostream& os)
{
    auto image = make_shared<Image>();
    if(!image->load(filename, os)){
        return false;
    }
    setImage(image);
    return true;
}


bool ImageItem::store(Archive& archive)
{
    return archive.writeFileInformation(this);
}


bool ImageItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}
