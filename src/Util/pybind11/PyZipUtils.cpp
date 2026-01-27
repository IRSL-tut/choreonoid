#include "PyUtil.h"
#include "../ZipArchiver.h"
#include <unordered_map>
#include <vector>

using namespace cnoid;
namespace py = pybind11;

namespace {

std::vector<unsigned char> bytesToVector(const py::bytes& bytes)
{
    std::string buffer = bytes;
    return std::vector<unsigned char>(buffer.begin(), buffer.end());
}

py::bytes vectorToBytes(const std::vector<unsigned char>& data)
{
    if(data.empty()){
        return py::bytes();
    }
    return py::bytes(reinterpret_cast<const char*>(data.data()), data.size());
}

py::dict zipMapToDict(const std::unordered_map<std::string, std::vector<unsigned char>>& zmap)
{
    py::dict d;
    for(const auto& kv : zmap){
        d[py::str(kv.first)] = vectorToBytes(kv.second);
    }
    return d;
}

}

namespace cnoid {

void exportPyZipUtils(py::module& m)
{
    py::class_<ZipArchiver> zipArchiver(m, "ZipArchiver");

    py::enum_<ZipArchiver::ErrorType>(zipArchiver, "ErrorType")
        .value("NoError", ZipArchiver::NoError)
        .value("ExistingZipFileRemovalError", ZipArchiver::ExistingZipFileRemovalError)
        .value("ZipFileCreationError", ZipArchiver::ZipFileCreationError)
        .value("DirectoryAdditionError", ZipArchiver::DirectoryAdditionError)
        .value("FileAdditionError", ZipArchiver::FileAdditionError)
        .value("ZipFileOpenError", ZipArchiver::ZipFileOpenError)
        .value("EntryExtractionError", ZipArchiver::EntryExtractionError)
        .value("DirectoryCreationError", ZipArchiver::DirectoryCreationError)
        .value("FileExtractionError", ZipArchiver::FileExtractionError)
        .export_values();

    zipArchiver
        .def(py::init<>())
        .def("createZipFile", &ZipArchiver::createZipFile,
             py::arg("zipFilename"), py::arg("directory"))
        .def("extractZipFile", &ZipArchiver::extractZipFile,
             py::arg("zipFilename"), py::arg("directory"))
        .def_property_readonly("extractedFiles", &ZipArchiver::extractedFiles)
        .def("errorType", &ZipArchiver::errorType)
        .def("systemErrorMessage", &ZipArchiver::systemErrorMessage)
        .def("errorMessage", &ZipArchiver::errorMessage)
        .def("topDir", &ZipArchiver::topDir)
        .def("extractSingleFile",
             [](ZipArchiver& self, const std::string& zipFilename){
                 std::vector<unsigned char> data;
                 bool ok = self.extractSingleFile(zipFilename, data);
                 return py::make_tuple(ok, vectorToBytes(data));
             },
             py::arg("zipFilename"))
        .def("extractZipFiles",
             [](ZipArchiver& self, const std::string& zipFilename){
                 std::unordered_map<std::string, std::vector<unsigned char>> zmap;
                 bool ok = self.extractZipFiles(zipFilename, zmap);
                 return py::make_tuple(ok, zipMapToDict(zmap));
             },
             py::arg("zipFilename"))
        .def("extractSingleFileData",
             [](ZipArchiver& self, const py::bytes& zipData){
                 std::vector<unsigned char> zipDataVec = bytesToVector(zipData);
                 std::vector<unsigned char> data;
                 bool ok = self.extractSingleFile(zipDataVec, data);
                 return py::make_tuple(ok, vectorToBytes(data));
             },
             py::arg("zipData"))
        .def("extractZipFilesData",
             [](ZipArchiver& self, const py::bytes& zipData){
                 std::vector<unsigned char> zipDataVec = bytesToVector(zipData);
                 std::unordered_map<std::string, std::vector<unsigned char>> zmap;
                 bool ok = self.extractZipFiles(zipDataVec, zmap);
                 return py::make_tuple(ok, zipMapToDict(zmap));
             },
             py::arg("zipData"))
        ;
}

}
