#ifndef CNOID_BASE_IMAGE_ITEM_H
#define CNOID_BASE_IMAGE_ITEM_H

#include "Item.h"
#include "ImageableItem.h"
#include <cnoid/Image>
#include <cnoid/Signal>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ImageItem : public Item, public ImageableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    ImageItem();
    ImageItem(const ImageItem& org);
    virtual ~ImageItem();

    // ImageableItem
    virtual const Image* getImage() override;
    virtual SignalProxy<void()> sigImageUpdated() override;

    const std::shared_ptr<Image>& image() const { return image_; }
    void setImage(std::shared_ptr<Image> image);
    bool loadImageFile(const std::string& filename, std::ostream& os = nullout());

protected:
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::shared_ptr<Image> image_;
    Signal<void()> sigImageUpdated_;
};

typedef ref_ptr<ImageItem> ImageItemPtr;

}

#endif
