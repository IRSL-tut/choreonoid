#ifndef CNOID_BASE_PROJECT_PACKER_H
#define CNOID_BASE_PROJECT_PACKER_H

#include <cnoid/MessageOut>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT ProjectPacker
{
public:
    ProjectPacker();
    virtual ~ProjectPacker();

    void setTopItemForPacking(Item* item);
    void addReferenceDirectory(std::string directory);
    void clearReferenceDirectories();
    void setUnpackingDirectory(const std::string& directory);
    const std::string& unpackingDirectory() const;
    bool packProjectToZipFile(const std::string& filename);
    bool packProjectToDirectory(const std::string& packingDirectory);

    /**
       Checks whether the file is a valid project pack file that contains a project
       file in a single top directory, without extracting the file.
       \param out_topDirectoryName The name of the top directory in the project pack
       is returned if this parameter is specified.
    */
    bool checkProjectPackFile(const std::string& filename, std::string* out_topDirectoryName = nullptr);

    bool loadPackedProject(const std::string& projectPackFile);
    bool unpackProject(const std::string& projectPackFile);

    //! The project file contained in the project pack unpacked last time
    std::string unpackedProjectFile() const;

    bool loadUnpackedProject(const std::string& projectFile);

protected:
    MessageOut* mout() { return mout_; }
    std::string getRelocatedFilePath(const std::string& path);

    virtual void getItemDependentFiles(Item* item, std::vector<std::string>& out_files);
    virtual Item* getPackingItem(Item* item);

private:
    MessageOut* mout_;
    
    class Impl;
    Impl* impl;
};

}

#endif
