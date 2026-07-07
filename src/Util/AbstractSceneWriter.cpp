#include "AbstractSceneWriter.h"
#include "SceneDrawables.h"
#include "UTF8.h"
#include "Format.h"
#include <filesystem>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class AbstractSceneWriter::Impl
{
public:
    struct CopiedFileInfo {
        string filename;
        bool copied;
    };

    // Original file to copied file
    unordered_map<string, CopiedFileInfo> imageFileMap;
    unordered_set<string> copiedImageFiles;

    // Image source data or image body to the file written from it
    unordered_map<const void*, CopiedFileInfo> writtenImageFileMap;

    bool writeImageFile(SgImage* image, const string& outputBaseDir, string& out_imageFile, ostream& os);
};

}


AbstractSceneWriter::AbstractSceneWriter()
{
    impl = new Impl;
}


AbstractSceneWriter::~AbstractSceneWriter()
{
    delete impl;
}


void AbstractSceneWriter::setMessageSink(std::ostream& os)
{
    os_ = &os;
}


void AbstractSceneWriter::clearImageFileInformation()
{
    impl->imageFileMap.clear();
    impl->copiedImageFiles.clear();
    impl->writtenImageFileMap.clear();
}


bool AbstractSceneWriter::outputImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_imageFile)
{
    if(image->hasUri()){
        return findOrCopyImageFile(image, outputBaseDir, out_imageFile);
    }
    return impl->writeImageFile(image, outputBaseDir, out_imageFile, os());
}


bool AbstractSceneWriter::Impl::writeImageFile
(SgImage* image, const string& outputBaseDir, string& out_imageFile, ostream& os)
{
    auto sourceData = image->sourceData();
    string ext;
    if(sourceData){
        auto& mimeType = sourceData->mimeType();
        if(mimeType == "image/png"){
            ext = ".png";
        } else if(mimeType == "image/jpeg"){
            ext = ".jpg";
        } else {
            // The data of an unsupported media type is not output as it is because the
            // resulting file cannot be loaded. The image is encoded into PNG instead.
            sourceData = nullptr;
        }
    }
    if(!sourceData){
        ext = ".png";
    }

    const void* key =
        sourceData ? static_cast<const void*>(sourceData) : static_cast<const void*>(&image->constImage());
    auto found = writtenImageFileMap.find(key);
    if(found != writtenImageFileMap.end()){
        auto& info = found->second;
        out_imageFile = toUTF8(info.filename);
        return info.copied;
    }
    auto& info = writtenImageFileMap[key];
    info.copied = false;

    if(!sourceData){
        auto& orgImage = image->constImage();
        if(orgImage.empty()){
            os << formatR(_("Warning: Texture image \"{0}\" is not output because it is empty."),
                          image->name()) << endl;
            return false;
        }
        if(orgImage.pixelType() != Image::UInt8){
            os << formatR(_("Warning: Texture image \"{0}\" is not output because its pixel format "
                            "is not supported."), image->name()) << endl;
            return false;
        }
    }

    // Determine the output file name from the image name
    string name = image->name();
    filesystem::path namePath(fromUTF8(name));
    auto nameExt = namePath.extension().string();
    std::transform(nameExt.begin(), nameExt.end(), nameExt.begin(), ::tolower);
    if(nameExt == ".png" || nameExt == ".jpg" || nameExt == ".jpeg"){
        name = toUTF8(namePath.stem().string());
    }
    string stem;
    stem.reserve(name.size());
    for(char c : name){
        bool valid = static_cast<unsigned char>(c) >= 0x20 && !strchr("\\/:*?\"<>|", c);
        stem += valid ? c : '_';
    }
    if(stem.empty()){
        stem = "image";
    }

    filesystem::path baseDirPath(fromUTF8(outputBaseDir));
    filesystem::path filePath(fromUTF8(stem + ext));
    filesystem::path absPath = baseDirPath / filePath;
    int counter = 2;
    while(true){
        auto inserted = copiedImageFiles.insert(absPath.string());
        if(inserted.second){
            break;
        }
        filePath = filesystem::path(fromUTF8(formatC("{0}-{1}{2}", stem, counter, ext)));
        ++counter;
        absPath = baseDirPath / filePath;
    }

    bool written = false;
    std::error_code ec;
    filesystem::create_directories(absPath.parent_path(), ec);
    if(ec){
        os << formatR(_("Warning: Texture image file \"{0}\" cannot be written: {1}"),
                      toUTF8(filePath.string()), ec.message()) << endl;
    } else if(sourceData){
        ofstream ofs(absPath, ios::binary);
        if(ofs){
            auto& data = sourceData->data();
            ofs.write(reinterpret_cast<const char*>(data.data()), data.size());
            ofs.close();
            written = ofs.good();
        }
        if(!written){
            os << formatR(_("Warning: Texture image file \"{0}\" cannot be written: {1}"),
                          toUTF8(filePath.string()), strerror(errno)) << endl;
        }
    } else {
        /*
           The pixels of the internal image are stored in the bottom-up order following
           the OpenGL convention, while the loaders load image files with flipping them
           vertically. The image must therefore be flipped to save it as a regular
           image file. Note that the save function outputs its own error message.
        */
        Image flippedImage(image->constImage());
        flippedImage.applyVerticalFlip();
        written = flippedImage.save(toUTF8(absPath.string()), os);
    }

    if(written){
        out_imageFile = toUTF8(filePath.string());
        info.filename = filePath.string();
        info.copied = true;
    }

    return written;
}


bool AbstractSceneWriter::findOrCopyImageFile(SgImage* image, const std::string& outputBaseDir, std::string& out_copiedFile)
{
    bool foundOrCopied = false;
    bool orgImageFileFound = false;
    std::error_code ec;
    
    auto uri = image->uri();
    if(uri.find("file://") == 0){
        uri = uri.substr(7);
    }
    filesystem::path filePath(fromUTF8(uri));

    if(filePath.is_absolute()){
        orgImageFileFound = filesystem::exists(filePath, ec);
        if(orgImageFileFound){
            out_copiedFile = uri;
            foundOrCopied = true;
        }
    } else if(image->hasAbsoluteUri()){
        auto& absUri = image->absoluteUri();
        if(absUri.find("file://") == 0){
            filesystem::path orgFilePath(fromUTF8(absUri.substr(7)));
            auto found = impl->imageFileMap.find(orgFilePath.string());
            if(found != impl->imageFileMap.end()){
                auto& info = found->second;
                out_copiedFile = toUTF8(info.filename);
                orgImageFileFound = true;
                foundOrCopied = info.copied;

            } else if(filesystem::exists(orgFilePath, ec)){
                orgImageFileFound = true;
                filesystem::path absPath;
                if(filePath.is_relative()){
                    absPath = filesystem::path(fromUTF8(outputBaseDir)) / filePath;
                } else {
                    absPath = filePath;
                }
                auto stem = filePath.stem().string();
                auto ext = filePath.extension().string();
                int counter = 2;
                while(true){
                    auto inserted = impl->copiedImageFiles.insert(absPath.string());
                    if(inserted.second){
                        break;
                    }
                    filePath = filePath.parent_path() / formatC("{0}-{1}{2}", stem, counter, ext);
                    ++counter;
                    if(filePath.is_relative()){
                        absPath = filesystem::path(fromUTF8(outputBaseDir)) / filePath;
                    } else {
                        absPath = filePath;
                    }
                }
                if(filesystem::equivalent(orgFilePath, absPath, ec)){
                    foundOrCopied = true;
                    out_copiedFile = toUTF8(filePath.string());
                } else {
                    filesystem::create_directories(absPath.parent_path(), ec);
                    if(!ec){
#if __cplusplus > 201402L
                        filesystem::copy_file(
                            orgFilePath, absPath, filesystem::copy_options::overwrite_existing, ec);
#else
                        filesystem::copy_file(
                            orgFilePath, absPath, filesystem::copy_option::overwrite_if_exists, ec);
#endif
                    }
                    if(!ec){
                        foundOrCopied = true;
                        out_copiedFile = toUTF8(filePath.string());
                    }
                }
                if(ec){
                    os() << formatR(_("Warning: Texture image file \"{0}\" cannot be copied: {1}"),
                                    uri, ec.message()) << endl;
                }

                auto& info = impl->imageFileMap[orgFilePath.string()];
                info.filename = filePath.string();
                info.copied = foundOrCopied;
            }
        }
    }
    
    if(!orgImageFileFound){
        os() << formatR(_("Warning: Texture image file \"{0}\" is not found."), uri) << endl;
    }

    return foundOrCopied;
}
