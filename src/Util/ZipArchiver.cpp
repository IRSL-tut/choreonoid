#include "ZipArchiver.h"
#include "FileUtil.h"
#include "UTF8.h"
#include "Format.h"
#include <cnoid/stdx/filesystem>
#include <zip.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace fs = stdx::filesystem;

namespace cnoid {

class ZipArchiver::Impl
{
public:
    ErrorType errorType;
    std::string systemErrorMessage;
    std::string errorMessage;
    std::vector<std::string> extractedFiles;
    std::string _topDir;
    Impl();
    bool createZipFile(const std::string& zipFilename, const std::string& directory);
    bool addDirectoryToZip(zip_t* zip, fs::path dirPath, const fs::path& srcTopDirPath, const fs::path& zipTopDirPath);
    bool extractZipFile(const std::string& zipFilename, const std::string& directory);
    bool extractFilesFromZipFile(
        zip_t* zip, const string& zipFilename, const fs::path&  zipFilePath, const fs::path& topDirPath);
    //IRSL
    bool extractSingleFileZip(zip_t *zip, const std::string& zipFilename, std::vector<unsigned char> &data);
    bool extractSingleFile(const std::string& zipFilename, std::vector<unsigned char> &data);
    bool extractSingleFile(const std::vector<unsigned char> &zip_data, std::vector<unsigned char> &data);
    bool extractZipFilesZip(zip_t *zip, const std::string& zipFilename, ZipFileMap &zmap);
    bool extractZipFiles(const std::string& zipFilename, ZipFileMap &zmap);
    bool extractZipFiles(const std::vector<unsigned char> &zip_data, ZipFileMap &zmap);
};

}


ZipArchiver::ZipArchiver()
{
    impl = new Impl;
}


ZipArchiver::Impl::Impl()
{
    errorType = NoError;
}


ZipArchiver::~ZipArchiver()
{
    delete impl;
}


ZipArchiver::ErrorType ZipArchiver::errorType() const
{
    return impl->errorType;
}


const std::string& ZipArchiver::systemErrorMessage() const
{
    return impl->systemErrorMessage;
}


const std::string& ZipArchiver::errorMessage() const
{
    return impl->errorMessage;
}

const std::string& ZipArchiver::topDir() const
{
    return impl->_topDir;
}

bool ZipArchiver::createZipFile(const std::string& zipFilename, const std::string& directory)
{
    return impl->createZipFile(zipFilename, directory);
}


bool ZipArchiver::Impl::createZipFile(const std::string& zipFilename, const std::string& directory)
{
    fs::path zipFilePath(fromUTF8(zipFilename));

    stdx::error_code ec;
    if(fs::exists(zipFilePath)){
        fs::remove(zipFilePath, ec);
        if(ec){
            errorType = ExistingZipFileRemovalError;
            systemErrorMessage = toUTF8(ec.message());
            errorMessage =
                formatR(_("The zip file \"{0}\" already exists and cannot be removed: {1}"),
                        zipFilename, systemErrorMessage);
            return false;
        }
    }
        
    int errorp;
    zip_t* zip = zip_open(zipFilePath.make_preferred().string().c_str(), ZIP_CREATE, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        errorType = ZipFileCreationError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage =
            formatR(_("Failed to create the zip file \"{0}\": {1}"),
                    zipFilename, systemErrorMessage);
        return false;
    }

    fs::path zipTopDirPath(zipFilePath.stem());
    fs::path dirPath(fromUTF8(directory));
    bool zipped = addDirectoryToZip(zip, dirPath, dirPath, zipTopDirPath);
    zip_close(zip);

    if(zipped){
        errorType = NoError;
        systemErrorMessage.clear();
        errorMessage.clear();
        
    } else {
        if(fs::exists(zipFilePath)){
            fs::remove(zipFilePath, ec);
        }
    }

    return zipped;
}


bool ZipArchiver::Impl::addDirectoryToZip
(zip_t* zip, fs::path currentSrcDirPath, const fs::path& srcTopDirPath, const fs::path& zipTopDirPath)
{
    auto relativePath = getRelativePath(currentSrcDirPath, srcTopDirPath);
    auto localDirPath = zipTopDirPath / relativePath->lexically_normal();
    auto localDirStr = toUTF8(localDirPath.generic_string());
    int index = zip_dir_add(zip, localDirStr.c_str(), ZIP_FL_ENC_UTF_8);
    if(index < 0){
        errorType = DirectoryAdditionError;
        systemErrorMessage = zip_strerror(zip);
        errorMessage =
            formatR(_("Failed to add directory \"{0}\" to the zip file: {1}"), localDirStr, systemErrorMessage);
        return false;
    }

    for(const fs::directory_entry& entry : fs::directory_iterator(currentSrcDirPath)){
        auto entryPath = entry.path();
        if(fs::is_directory(entryPath)){
            if(!addDirectoryToZip(zip, entryPath, srcTopDirPath, zipTopDirPath)){
                return false;
            }
        } else {
            auto localPath = zipTopDirPath / (*getRelativePath(entryPath, srcTopDirPath));
            auto localPathStr = toUTF8(localPath.generic_string());
            auto sourcePath = toUTF8(entryPath.make_preferred().string());
            zip_source_t* source = zip_source_file(zip, sourcePath.c_str(), 0, 0);
            if(!source){
                errorType = FileAdditionError;
                systemErrorMessage = zip_strerror(zip);
                errorMessage =
                    formatR(_("Failed to add file \"{0}\" to the zip file: {1}"),
                            localPathStr, systemErrorMessage);
                return false;
            }
            int index = zip_file_add(zip, localPathStr.c_str(), source, ZIP_FL_ENC_UTF_8);
            if(index < 0){
                zip_source_free(source);
                errorType = FileAdditionError;
                systemErrorMessage = zip_strerror(zip);
                errorMessage =
                    formatR(_("Failed to add file \"{0}\" to the zip file: {1}"),
                            localPathStr, systemErrorMessage);
                return false;
            }
            // The deflate compression is applied by default. The following code is not necessary.
            /*
            if(zip_set_file_compression(zip, index, ZIP_CM_DEFLATE, 0) < 0){
                zip_source_free(source);
                mout->putErrorln(
                    formatR(_("Failed to compress file \"{0}\": {1}"),
                            localPathStr, zip_strerror(zip)));
                return false;
            }
            */
        }
    }

    return true;
}


bool ZipArchiver::extractZipFile(const std::string& zipFilename, const std::string& directory)
{
    return impl->extractZipFile(zipFilename, directory);
}


bool ZipArchiver::Impl::extractZipFile(const std::string& zipFilename, const std::string& directory)
{
    _topDir.clear();
    extractedFiles.clear();

    fs::path zipFilePath(fromUTF8(zipFilename));

    int errorp;
    zip_t* zip = zip_open(zipFilePath.make_preferred().string().c_str(), ZIP_RDONLY, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = 
            formatR(_("Failed to open the zip file \"{0}\": {1}"),
                    zipFilename, systemErrorMessage);;
        return false;
    }

    fs::path extractionDirPath(fromUTF8(directory));
    bool extracted = extractFilesFromZipFile(zip, zipFilename, zipFilePath, extractionDirPath);
    zip_close(zip);

    if(extracted){
        errorType = NoError;
        systemErrorMessage.clear();
        errorMessage.clear();
    }

    return extracted;
}


bool ZipArchiver::Impl::extractFilesFromZipFile
(zip_t* zip, const string& zipFilename, const fs::path& zipFilePath, const fs::path& topDirPath)
{
    vector<unsigned char> buf(1024 * 1024);
    stdx::error_code ec;
    int numEntries = zip_get_num_entries(zip, 0);
    {// check top-dir
        zip_stat_t stat;
        if (zip_stat_index(zip, 0, 0, &stat) >= 0) {
            string dname(stat.name);
            if (dname[dname.size() - 1] == '/') {
                _topDir = dname;
            }
        }
    }
    for(int i = 0; i < numEntries; ++i){
        zip_stat_t stat;
        if(zip_stat_index(zip, i, 0, &stat) < 0){
            errorType = EntryExtractionError;
            systemErrorMessage.clear();
            errorMessage = formatR(_("Entry {0} in the zip file \"{1}\" cannot be extracted."), i, zipFilename);
            return false;
        } else {
            string name(stat.name);
            auto entryPath = topDirPath / fromUTF8(name);
            if(name[name.size() - 1] == '/'){
                bool failed = false;
                if(fs::exists(entryPath)){
                    fs::remove_all(entryPath, ec);
                    if(ec){
                        failed = true;
                    }
                }
                if(!failed){
                    fs::create_directories(entryPath, ec);
                    if(ec){
                        failed = true;
                    }
                }
                if(failed){
                    errorType = DirectoryCreationError;
                    systemErrorMessage = toUTF8(ec.message());
                    errorMessage =
                        formatR(_("Directory \"{0}\" in the zip file \"{1}\" cannot be created: {2}"),
                                name, zipFilename, systemErrorMessage);
                    return false;
                }
            } else {
                bool failed = false;
                zip_file_t* zf = zip_fopen_index(zip, i, 0);
                if(!zf){
                    failed = true;
                } else {
                    FILE* file = fopen(entryPath.make_preferred().string().c_str(), "wb");
                    if(!file){
                        failed = true;
                    } else {
                        long long sum = 0;
                        while(sum < stat.size){
                            int len = zip_fread(zf, buf.data(), buf.size());
                            if(len < 0){
                                failed = true;
                            } else {
                                if(fwrite(buf.data(), sizeof(unsigned char), len, file) < len){
                                    failed = true;
                                    break;
                                }
                                sum += len;
                            }
                        }
                        fclose(file);

                        extractedFiles.push_back(toUTF8(entryPath.generic_string()));
                    }
                }
                if(failed){
                    errorType = FileExtractionError;
                    systemErrorMessage.clear();
                    errorMessage = 
                        formatR(_("File \"{0}\" in the zip file \"{1}\" cannot be extracted."),
                                name, zipFilename);
                    return false;
                }
                    
                zip_fclose(zf);
            }
        }
    }

    return true;
}


const std::vector<std::string>& ZipArchiver::extractedFiles() const
{
    return impl->extractedFiles;
}
/// IRSL
bool ZipArchiver::Impl::extractSingleFileZip(zip_t *zip, const std::string& zipFilename, std::vector<unsigned char> &data)
{
    int numEntries = zip_get_num_entries(zip, 0);
    for(int i = 0; i < numEntries; ++i) {
        zip_stat_t stat;
        if (zip_stat_index(zip, i, 0, &stat) < 0) {
            errorType = EntryExtractionError;
            systemErrorMessage.clear();
            errorMessage = formatR(_("Entry {0} in the zip file \"{1}\" cannot be extracted."), i, zipFilename);
            break;
        } else {
            string name(stat.name);
            if (name[name.size() - 1] == '/') {
                // do nothing
            } else {
                bool failed = false;
                zip_file_t* zf = zip_fopen_index(zip, i, 0);
                if (!zf) {
                    //
                } else {
                    data.resize(stat.size);
                    int len = zip_fread(zf, data.data(), data.size());
                    zip_fclose(zf);
                    if (len >= 0 && len == stat.size) {
                        return true;
                    }
                }
                errorType = FileExtractionError;
                systemErrorMessage.clear();
                errorMessage =
                formatR(_("File \"{0}\" in the zip file \"{1}\" cannot be extracted."),
                        name, zipFilename);
                break;
            }
        }
    } // for
    zip_close(zip);
    return false;
}
bool ZipArchiver::Impl::extractSingleFile(const std::string& zipFilename, std::vector<unsigned char> &data)
{
    fs::path zipFilePath(fromUTF8(zipFilename));

    int errorp;
    zip_t* zip = zip_open(zipFilePath.make_preferred().string().c_str(), ZIP_RDONLY, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = 
            formatR(_("Failed to open the zip file \"{0}\": {1}"),
                    zipFilename, systemErrorMessage);;
        return false;
    }

    return extractSingleFileZip(zip, zipFilename, data);
}
bool ZipArchiver::Impl::extractSingleFile(const std::vector<unsigned char> &zip_data, std::vector<unsigned char> &data)
{
    zip_error_t error;
    std::string zipFilename("<data>");
    //
    zip_source_t *source = zip_source_buffer_create(zip_data.data(), zip_data.size(), 0, &error);
    if (!source) {
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = formatR(_("Failed to create source buffer : {0}"), systemErrorMessage);
        return false;
    }
    //
    zip_t* zip = zip_open_from_source(source, 0, &error);
    if (!zip) {
        zip_source_free(source);
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = formatR(_("Failed to open from source : {0}"), systemErrorMessage);
        return false;
    }
    return extractSingleFileZip(zip, zipFilename, data);
}
bool ZipArchiver::extractSingleFile(const std::string& zipFilename, std::vector<unsigned char> &data)
{
    return impl->extractSingleFile(zipFilename, data);
}
bool ZipArchiver::extractSingleFile(const std::vector<unsigned char> &zip_data, std::vector<unsigned char> &data)
{
    return impl->extractSingleFile(zip_data, data);
}
bool ZipArchiver::Impl::extractZipFilesZip(zip_t *zip, const std::string& zipFilename, ZipFileMap &zmap)
{
    int numEntries = zip_get_num_entries(zip, 0);
    bool res = true;
    {// check top-dir
        zip_stat_t stat;
        if (zip_stat_index(zip, 0, 0, &stat) >= 0) {
            string dname(stat.name);
            if (dname[dname.size() - 1] == '/') {
                _topDir = dname;
            }
        }
    }
    for(int i = 0; i < numEntries; ++i) {
        zip_stat_t stat;
        if (zip_stat_index(zip, i, 0, &stat) < 0) {
            errorType = EntryExtractionError;
            systemErrorMessage.clear();
            errorMessage = formatR(_("Entry {0} in the zip file \"{1}\" cannot be extracted."), i, zipFilename);
            break;
        } else {
            string name(stat.name);
            if (name[name.size() - 1] == '/') {
                // do nothing
            } else {
                zip_file_t* zf = zip_fopen_index(zip, i, 0);
                if (!zf) {
                    res = false;
                    break;
                } else {
                    std::vector<unsigned char> data;
                    data.resize(stat.size);
                    int len = zip_fread(zf, data.data(), data.size());
                    zip_fclose(zf);
                    if (len >= 0 && len == stat.size) {
                        zmap.emplace(name, data);
                    } else {
                        errorType = FileExtractionError;
                        systemErrorMessage.clear();
                        errorMessage =
                        formatR(_("File \"{0}\" in the zip file \"{1}\" cannot be extracted."),
                                name, zipFilename);
                        res = false;
                        break;
                    }
                }
            }
        }
    } // for
    zip_close(zip);
    return res;
}
bool ZipArchiver::Impl::extractZipFiles(const std::string& zipFilename, ZipFileMap &zmap)
{
    _topDir.clear();
    fs::path zipFilePath(fromUTF8(zipFilename));
    int errorp;
    zip_t* zip = zip_open(zipFilePath.make_preferred().string().c_str(), ZIP_RDONLY, &errorp);
    if(!zip){
        zip_error_t error;
        zip_error_init_with_code(&error, errorp);
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = 
            formatR(_("Failed to open the zip file \"{0}\": {1}"),
                    zipFilename, systemErrorMessage);;
        return false;
    }

    return extractZipFilesZip(zip, zipFilename, zmap);
}
bool ZipArchiver::Impl::extractZipFiles(const std::vector<unsigned char> &zip_data, ZipFileMap &zmap)
{
    _topDir.clear();
    zip_error_t error;
    std::string zipFilename("<data>");
    //
    zip_source_t *source = zip_source_buffer_create(zip_data.data(), zip_data.size(), 0, &error);
    if (!source) {
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = formatR(_("Failed to create source buffer : {0}"), systemErrorMessage);
        return false;
    }
    //
    zip_t* zip = zip_open_from_source(source, 0, &error);
    if (!zip) {
        zip_source_free(source);
        errorType = ZipFileOpenError;
        systemErrorMessage = zip_error_strerror(&error);
        errorMessage = formatR(_("Failed to open from source : {0}"), systemErrorMessage);
        return false;
    }
    return extractZipFilesZip(zip, zipFilename, zmap);
}
bool ZipArchiver::extractZipFiles(const std::string& zipFilename, ZipFileMap &zmap)
{
    return impl->extractZipFiles(zipFilename, zmap);
}
bool ZipArchiver::extractZipFiles(const std::vector<unsigned char> &zip_data, ZipFileMap &zmap)
{
    return impl->extractZipFiles(zip_data, zmap);
}
