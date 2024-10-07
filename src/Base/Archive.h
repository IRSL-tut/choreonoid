#ifndef CNOID_BASE_ARCHIVE_H
#define CNOID_BASE_ARCHIVE_H

#include <cnoid/ValueTree>
#include <string>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class View;
class ViewManager;
class ProjectManager;
class FilePathVariableProcessor;
class ArchiveSharedData;
class MessageOut;

class CNOID_EXPORT Archive : public Mapping
{
public:
    Archive();
    Archive(int line, int column);
    virtual ~Archive();

    void initSharedInfo(
        const std::string& projectFile, bool isSubProject, MessageOut* mout, bool isSavingProjectAsBackup);
    void inheritSharedInfoFrom(Archive& archive);

    /**
       \note This function can be used recursively in a called function to do additional processes
       which will be done after all the previously added processes.
    */
    void addProcessOnSubTreeRestored(const std::function<void()>& func) const;

    void addProcessOnSubTreeRestored(Item* item, const std::function<void()>& func) const;
    
    void addPostProcess(const std::function<void()>& func, int priority = 0) const;
    void addFinalProcess(const std::function<void()>& func) const;

    Archive* findSubArchive(const std::string& name);
    const Archive* findSubArchive(const std::string& name) const;
    bool forSubArchive(const std::string& name, std::function<bool(const Archive& archive)> func) const;
    Archive* openSubArchive(const std::string& name);
    Archive* subArchive(Mapping* node);
    const Archive* subArchive(Mapping* node) const;

    ValueNodePtr getItemIdNode(const Item* item) const;
    Item* findItem(const ValueNode* id) const;
    
    int getViewId(const View* view) const;
    View* findView(int id) const;

    void clearIds();
        
    template<class ItemType> inline ItemType* findItem(ValueNode* id) const {
        return dynamic_cast<ItemType*>(findItem(id));
    }

    void writeItemId(const std::string& key, Item* item);

    template<class ItemType> inline ItemType* findItem(const std::string& key) const {
        ValueNode* id = find(key);
        return id->isValid() ? findItem<ItemType>(id) : nullptr;
    }

    std::string resolveRelocatablePath(const std::string& relocatable, bool doAbsolutize = true) const;
    bool readRelocatablePath(const std::string& key, std::string& out_value) const;
    std::string readItemFilePath() const;

    bool loadFileTo(Item* item, bool& out_hasFileInformation) const;
    bool loadFileTo(Item* item) const;
    bool loadFileTo(Item* item, const std::string& filepath) const;

    std::string getRelocatablePath(const std::string& path) const;
    bool writeRelocatablePath(const std::string& key, const std::string& path);
    bool writeFileInformation(Item* item);
    bool saveItemToFile(Item* item);

    Item* currentParentItem() const;
    std::string projectDirectory() const;
    FilePathVariableProcessor* filePathVariableProcessor() const;
    bool isSavingProjectAsBackup() const;
    MessageOut* mout() const;

    [[deprecated("Use resolveRelocatablePath(path, false).")]]
    std::string expandPathVariables(const std::string& path) const {
        return resolveRelocatablePath(path, false);
    }
    [[deprecated("Use loadFileTo(Item* item, const std::string& filepath)")]]
    bool loadFileTo(const std::string& filepath, Item* item) const {
        return loadFileTo(item, filepath);
    }
    [[deprecated]]
    bool loadItemFile(Item* item, const std::string& fileNameKey, const std::string& fileFormatKey = std::string()) const;
    

private:
    ref_ptr<ArchiveSharedData> shared;

    Item* findItem(int id) const;
    void setCurrentItem(Item* item);
    void setCurrentParentItem(Item* parentItem);
    static Archive* invalidArchive();
    void registerItemId(const Item* item, int id);
    void registerViewId(const View* view, int id);
    void callProcessesOnSubTreeRestored(Item* item);
    void callPostProcesses();
    static void callFinalProcesses();

    friend class ItemTreeArchiver;
    friend class ViewManager;
    friend class ProjectManager;
};

typedef ref_ptr<Archive> ArchivePtr;

}

#endif
