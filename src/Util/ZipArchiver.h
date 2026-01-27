#ifndef CNOID_UTIL_ZIP_ARCHIVER_H
#define CNOID_UTIL_ZIP_ARCHIVER_H

#include "exportdecl.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace cnoid {

class CNOID_EXPORT ZipArchiver
{
    typedef std::string ZipFileKey;
    typedef std::vector<unsigned char> ZipFileData;
    typedef std::unordered_map<ZipFileKey, ZipFileData> ZipFileMap;

public:
    ZipArchiver();
    virtual ~ZipArchiver();

    bool createZipFile(const std::string& zipFilename, const std::string& directory);
    bool extractZipFile(const std::string& zipFilename, const std::string& directory);
    const std::vector<std::string>& extractedFiles() const;

    enum ErrorType {
        NoError,
        ExistingZipFileRemovalError,
        ZipFileCreationError,
        DirectoryAdditionError,
        FileAdditionError,
        ZipFileOpenError,
        EntryExtractionError,
        DirectoryCreationError,
        FileExtractionError
    };

    ErrorType errorType() const;
    const std::string& systemErrorMessage() const;
    const std::string& errorMessage() const;

    //IRSL
    bool extractSingleFile(const std::string& zipFilename, std::vector<unsigned char> &data);
    bool extractZipFiles(const std::string& zipFilename, ZipFileMap &zmap);
    bool extractSingleFile(const std::vector<unsigned char> &zip_data, std::vector<unsigned char> &data);
    bool extractZipFiles(const std::vector<unsigned char> &zip_data, ZipFileMap &zmap);
    const std::string &topDir() const;
private:
    class Impl;
    Impl* impl;
};

}

#endif 
