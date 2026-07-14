#include "GLTFSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "SceneUtil.h"
#include "MeshUtil.h"
#include "YAMLReader.h"
#include "ValueTree.h"
#include "EigenArchive.h"
#include "ImageIO.h"
#include "MessageOut.h"
#include "MeshFilter.h"
#include "NullOut.h"
#include "UTF8.h"
#include "Format.h"
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <optional>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace filesystem = std::filesystem;

namespace {

// glTF 2.0 component type constants
constexpr int GLTF_BYTE = 5120;
constexpr int GLTF_UNSIGNED_BYTE = 5121;
constexpr int GLTF_SHORT = 5122;
constexpr int GLTF_UNSIGNED_SHORT = 5123;
constexpr int GLTF_UNSIGNED_INT = 5125;
constexpr int GLTF_FLOAT = 5126;

// glTF 2.0 primitive mode constants
constexpr int GLTF_POINTS = 0;
constexpr int GLTF_LINES = 1;
constexpr int GLTF_LINE_LOOP = 2;
constexpr int GLTF_LINE_STRIP = 3;
constexpr int GLTF_TRIANGLES = 4;
constexpr int GLTF_TRIANGLE_STRIP = 5;
constexpr int GLTF_TRIANGLE_FAN = 6;

// glTF 2.0 sampler wrap mode constants
constexpr int GLTF_CLAMP_TO_EDGE = 33071;

// GLB container constants
constexpr uint32_t GLB_MAGIC = 0x46546C67; // "glTF"
constexpr uint32_t GLB_CHUNK_JSON = 0x4E4F534A; // "JSON"
constexpr uint32_t GLB_CHUNK_BIN = 0x004E4942; // "BIN"

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            { "gltf", "glb" },
            []() -> shared_ptr<AbstractSceneLoader> {
                return make_shared<GLTFSceneLoader>(); });
    }
} registration;

class LoadingException
{
public:
    LoadingException(const std::string& message) : message(message) { }
    std::string message;
};

int componentSizeOf(int componentType)
{
    switch(componentType){
    case GLTF_BYTE:
    case GLTF_UNSIGNED_BYTE:
        return 1;
    case GLTF_SHORT:
    case GLTF_UNSIGNED_SHORT:
        return 2;
    case GLTF_UNSIGNED_INT:
    case GLTF_FLOAT:
        return 4;
    default:
        return 0;
    }
}

int numComponentsOf(const string& type)
{
    if(type == "SCALAR") return 1;
    if(type == "VEC2") return 2;
    if(type == "VEC3") return 3;
    if(type == "VEC4") return 4;
    if(type == "MAT2") return 4;
    if(type == "MAT3") return 9;
    if(type == "MAT4") return 16;
    return 0;
}

bool decodeBase64(const string& text, size_t pos, vector<uint8_t>& out)
{
    static const int8_t table[256] = {
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,
        52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,
        -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
        15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,
        -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
        41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1
    };
    out.clear();
    out.reserve((text.size() - pos) / 4 * 3);
    uint32_t bits = 0;
    int numBits = 0;
    for(size_t i = pos; i < text.size(); ++i){
        const char c = text[i];
        if(c == '='){
            break;
        }
        const int8_t value = table[static_cast<uint8_t>(c)];
        if(value < 0){
            return false;
        }
        bits = (bits << 6) | value;
        numBits += 6;
        if(numBits >= 8){
            numBits -= 8;
            out.push_back(static_cast<uint8_t>((bits >> numBits) & 0xff));
        }
    }
    return true;
}

string detectImageMimeType(const uint8_t* data, size_t size)
{
    if(size >= 4 && data[0] == 0x89 && data[1] == 'P' && data[2] == 'N' && data[3] == 'G'){
        return "image/png";
    }
    if(size >= 3 && data[0] == 0xFF && data[1] == 0xD8 && data[2] == 0xFF){
        return "image/jpeg";
    }
    return string();
}

// Decode the percent-encoding ("%20" etc.) of a URI
string decodeUri(const string& uri)
{
    string decoded;
    decoded.reserve(uri.size());
    for(size_t i = 0; i < uri.size(); ++i){
        if(uri[i] == '%' && i + 2 < uri.size() &&
           isxdigit(static_cast<unsigned char>(uri[i + 1])) &&
           isxdigit(static_cast<unsigned char>(uri[i + 2]))){
            decoded.push_back(static_cast<char>(stoi(uri.substr(i + 1, 2), nullptr, 16)));
            i += 2;
        } else {
            decoded.push_back(uri[i]);
        }
    }
    return decoded;
}

}

namespace cnoid {

class GLTFSceneLoader::Impl
{
public:
    GLTFSceneLoader* self;
    ostream* os_;
    ostream& os() { return *os_; }
    MessageOutPtr mout;
    ImageIO imageIO;
    MeshFilter meshFilter;
    YAMLReader reader;
    vector<filesystem::path> imageSearchDirectories;

    // Loading context
    filesystem::path directory; // directory of the file being loaded
    MappingPtr gltf; // top-level JSON object
    Listing* accessorList;
    Listing* bufferViewList;
    Listing* bufferList;
    Listing* meshList;
    Listing* nodeList;
    Listing* materialList;
    Listing* textureList;
    Listing* imageList;
    Listing* samplerList;
    vector<uint8_t> glbBinChunk;
    bool hasGlbBinChunk;
    unordered_map<int, vector<uint8_t>> bufferDataMap;
    unordered_map<int, SgGroupPtr> meshGroupMap;
    unordered_map<int, SgMaterialPtr> materialMap;
    unordered_map<int, SgTexturePtr> textureMap;
    unordered_map<int, SgImagePtr> imageMap;
    SgMaterialPtr defaultMaterial;
    unordered_set<string> oneTimeWarnings;

    /*
       The transform accumulated from the node where a reflection (a negative
       determinant matrix or a negative scale component) is found. While this is
       active, the transforms are baked into the mesh vertices instead of being
       kept in the scene graph, because the downstream subsystems assuming
       rotation-only transforms cannot correctly handle a reflection. This follows
       the implementation note of the glTF 2.0 specification that the winding
       order should be inverted for a transform whose determinant is negative.
    */
    std::optional<Affine3f> bakeTransform;

    Impl(GLTFSceneLoader* self);
    void clearLoadingContext();
    void warnOnce(const string& message);
    SgNode* load(const string& filename);
    string readGlbJsonText(ifstream& file, const string& filename);
    SgNode* loadScene();
    SgNode* convertNode(int nodeIndex, int depth);
    SgGroup* getOrCreateMesh(int meshIndex);
    SgNode* convertPrimitive(Mapping* primitive, const string& meshName, int primitiveIndex);
    SgNode* convertPlotPrimitive(Mapping* primitive, int mode, const string& meshName, int primitiveIndex);
    bool readPositionAttribute(Mapping* primitive, vector<float>& out_values);
    void convertTriangleIndices(SgMesh* mesh, SgIndexArray& indices, int mode, int numVertices);
    SgMaterial* getOrCreateMaterial(int materialIndex);
    SgMaterial* convertMaterial(Mapping* materialInfo);
    SgTexture* getOrCreateTexture(int textureIndex);
    SgImage* getOrCreateImage(int imageIndex);
    const vector<uint8_t>& getBufferData(int bufferIndex);
    Mapping* getListingElement(Listing* listing, int index, const char* listingName);
    void readAccessorAsFloat(int accessorIndex, int expectedNumComponents, vector<float>& out);
    void readAccessorAsIndex(int accessorIndex, SgIndexArray& out);
    void getAccessorDataPointer(
        Mapping* accessor, int numComponents,
        const uint8_t*& out_pointer, int& out_stride, int& out_count, int& out_componentType,
        bool& out_normalized);
};

}


GLTFSceneLoader::GLTFSceneLoader()
{
    impl = new Impl(this);
}


GLTFSceneLoader::Impl::Impl(GLTFSceneLoader* self)
    : self(self)
{
    os_ = &nullout();

    /*
       The image data of glTF is loaded upside down and the V components of the texture
       coordinates are inverted in loading them so that the data follow the convention
       of the texture image orientation and the texture coordinate origin (bottom-left)
       used in Choreonoid, which is same as that of OBJ and VRML.
       (The origin of the texture coordinates is top-left in glTF.)
    */
    imageIO.setUpsideDown(true);
}


GLTFSceneLoader::~GLTFSceneLoader()
{
    delete impl;
}


void GLTFSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void GLTFSceneLoader::addImageSearchDirectory(const std::string& directory)
{
    impl->imageSearchDirectories.emplace_back(fromUTF8(directory));
}


void GLTFSceneLoader::clearImageSearchDirectories()
{
    impl->imageSearchDirectories.clear();
}


SgNode* GLTFSceneLoader::load(const std::string& filename)
{
    return impl->load(filename);
}


void GLTFSceneLoader::Impl::clearLoadingContext()
{
    gltf.reset();
    accessorList = nullptr;
    bufferViewList = nullptr;
    bufferList = nullptr;
    meshList = nullptr;
    nodeList = nullptr;
    materialList = nullptr;
    textureList = nullptr;
    imageList = nullptr;
    samplerList = nullptr;
    glbBinChunk.clear();
    hasGlbBinChunk = false;
    bufferDataMap.clear();
    meshGroupMap.clear();
    materialMap.clear();
    textureMap.clear();
    imageMap.clear();
    defaultMaterial.reset();
    oneTimeWarnings.clear();
    bakeTransform.reset();
    reader.clearDocuments();
}


void GLTFSceneLoader::Impl::warnOnce(const string& message)
{
    if(oneTimeWarnings.insert(message).second){
        os() << message << endl;
    }
}


SgNode* GLTFSceneLoader::Impl::load(const string& filename)
{
    clearLoadingContext();
    mout = new MessageOut(os());

    SgNode* scene = nullptr;

    try {
        filesystem::path filepath(fromUTF8(filename));
        directory = filepath.parent_path();

        ifstream file(filepath, ios::binary);
        if(!file){
            throw LoadingException(formatR(_("The file cannot be opened. {0}"), strerror(errno)));
        }

        string jsonText;
        uint32_t magic = 0;
        file.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        file.seekg(0);
        if(magic == GLB_MAGIC){
            jsonText = readGlbJsonText(file, filename);
        } else {
            ostringstream ss;
            ss << file.rdbuf();
            jsonText = ss.str();
        }
        file.close();

        if(!reader.parse(jsonText)){
            throw LoadingException(
                formatR(_("The glTF JSON data cannot be parsed. {0}"), reader.errorMessage()));
        }
        gltf = reader.document()->toMapping();

        auto asset = gltf->findMapping("asset");
        if(!asset->isValid()){
            throw LoadingException(_("The file does not have the glTF asset information."));
        }
        string version = asset->get("version", "");
        if(version.compare(0, 2, "2.") != 0){
            throw LoadingException(
                formatR(_("glTF version {0} is not supported."), version));
        }

        if(auto required = gltf->findListing("extensionsRequired"); required->isValid()){
            static const unordered_set<string> supportedExtensions = {
                "KHR_materials_emissive_strength",
                "KHR_materials_specular"
            };
            for(auto& node : *required){
                string extension = node->toString();
                if(supportedExtensions.find(extension) == supportedExtensions.end()){
                    os() << formatR(
                        _("Warning: Required glTF extension \"{0}\" is not supported. "
                          "The model may not be loaded correctly."), extension) << endl;
                }
            }
        }

        accessorList = gltf->findListing("accessors");
        bufferViewList = gltf->findListing("bufferViews");
        bufferList = gltf->findListing("buffers");
        meshList = gltf->findListing("meshes");
        nodeList = gltf->findListing("nodes");
        materialList = gltf->findListing("materials");
        textureList = gltf->findListing("textures");
        imageList = gltf->findListing("images");
        samplerList = gltf->findListing("samplers");

        scene = loadScene();

        /*
           The coordinate values are loaded as they are although the glTF specification
           defines the Y-up coordinate system. Following the convention of the other
           mesh file loaders, the coordinate conversion is only applied when it is
           explicitly specified with the loader hints.
        */
        scene = self->insertTransformNodesToAdjustLengthUnitAndUpperAxis(scene);
        self->storeLengthUnitAndUpperAxisHintsAsMetadata(scene);
    }
    catch(const LoadingException& ex){
        os() << formatR(_("glTF file \"{0}\" cannot be loaded: {1}"), filename, ex.message) << endl;
        scene = nullptr;
    }
    catch(const ValueNode::Exception& ex){
        os() << formatR(_("glTF file \"{0}\" cannot be loaded: {1}"), filename, ex.message()) << endl;
        scene = nullptr;
    }

    clearLoadingContext();
    mout.reset();

    return scene;
}


string GLTFSceneLoader::Impl::readGlbJsonText(ifstream& file, const string& filename)
{
    uint32_t header[3]; // magic, version, length
    file.read(reinterpret_cast<char*>(header), sizeof(header));
    if(!file){
        throw LoadingException(_("Invalid GLB header."));
    }
    if(header[1] != 2){
        throw LoadingException(
            formatR(_("GLB container version {0} is not supported."), header[1]));
    }

    string jsonText;
    const uint32_t totalLength = header[2];
    uint32_t pos = sizeof(header);
    while(pos + 8 <= totalLength){
        uint32_t chunkHeader[2]; // length, type
        file.read(reinterpret_cast<char*>(chunkHeader), sizeof(chunkHeader));
        if(!file){
            break;
        }
        pos += 8;
        const uint32_t chunkLength = chunkHeader[0];
        if(pos + chunkLength > totalLength){
            throw LoadingException(_("Invalid GLB chunk size."));
        }
        if(chunkHeader[1] == GLB_CHUNK_JSON && jsonText.empty()){
            jsonText.resize(chunkLength);
            file.read(&jsonText[0], chunkLength);
        } else if(chunkHeader[1] == GLB_CHUNK_BIN && !hasGlbBinChunk){
            glbBinChunk.resize(chunkLength);
            file.read(reinterpret_cast<char*>(glbBinChunk.data()), chunkLength);
            hasGlbBinChunk = true;
        } else {
            file.seekg(chunkLength, ios::cur);
        }
        if(!file){
            throw LoadingException(_("Invalid GLB chunk data."));
        }
        pos += chunkLength;
    }
    if(jsonText.empty()){
        throw LoadingException(_("The GLB file does not have a JSON chunk."));
    }
    return jsonText;
}


SgNode* GLTFSceneLoader::Impl::loadScene()
{
    auto sceneList = gltf->findListing("scenes");
    if(!sceneList->isValid() || sceneList->empty()){
        throw LoadingException(_("The file does not have any scenes."));
    }
    int sceneIndex = gltf->get("scene", 0);
    if(sceneIndex < 0 || sceneIndex >= sceneList->size()){
        throw LoadingException(formatR(_("Scene {0} is not found."), sceneIndex));
    }
    if(sceneList->size() >= 2){
        os() << formatR(_("The file has {0} scenes, but only the default scene is loaded."),
                        sceneList->size()) << endl;
    }
    auto sceneInfo = sceneList->at(sceneIndex)->toMapping();

    auto sceneGroup = new SgGroup;
    string name;
    if(sceneInfo->read("name", name)){
        sceneGroup->setName(name);
    }
    if(auto rootNodes = sceneInfo->findListing("nodes"); rootNodes->isValid()){
        for(auto& node : *rootNodes){
            if(auto sgNode = convertNode(node->toInt(), 0)){
                sceneGroup->addChild(sgNode);
            }
        }
    }

    return sceneGroup;
}


SgNode* GLTFSceneLoader::Impl::convertNode(int nodeIndex, int depth)
{
    if(depth > 512){
        throw LoadingException(_("The node hierarchy is too deep."));
    }
    auto nodeInfo = getListingElement(nodeList, nodeIndex, "nodes");

    SgGroupPtr group;
    SgGroup* childContainer; // just an alias of the node to which the children are added

    /*
       When the node transform contains a reflection (a matrix whose determinant is
       negative or a TRS with a negative scale component), the node and its
       descendants are converted into plain groups and the accumulated transform is
       baked into the mesh vertices in getOrCreateMesh.
    */
    std::optional<Affine3f> prevBakeTransform = bakeTransform;

    if(auto matrix = nodeInfo->findListing("matrix"); matrix->isValid()){
        if(matrix->size() != 16){
            throw LoadingException(formatR(_("Node {0} has an invalid matrix."), nodeIndex));
        }
        Affine3 T;
        for(int col = 0; col < 4; ++col){
            for(int row = 0; row < 4; ++row){
                // The glTF matrix is stored in the column-major order
                T.matrix()(row, col) = matrix->at(col * 4 + row)->toDouble();
            }
        }
        if(bakeTransform || T.linear().determinant() < 0.0){
            if(bakeTransform){
                bakeTransform = (*bakeTransform) * T.cast<float>();
            } else {
                bakeTransform = T.cast<float>();
            }
            group = new SgGroup;
            childContainer = group;
        } else {
            SgGroupPtr top;
            SgGroupPtr inner;
            std::tie(top, inner) = createTransformNodeSet(T);
            group = top;
            childContainer = inner;
        }
    } else {
        Vector3 translation = Vector3::Zero();
        read(nodeInfo, "translation", translation);
        Quaterniond q = Quaterniond::Identity();
        if(auto rotation = nodeInfo->findListing("rotation"); rotation->isValid() && rotation->size() == 4){
            q = Quaterniond(
                rotation->at(3)->toDouble(),  // w
                rotation->at(0)->toDouble(),  // x
                rotation->at(1)->toDouble(),  // y
                rotation->at(2)->toDouble()); // z
            q.normalize();
        }
        Vector3 scale = Vector3::Ones();
        read(nodeInfo, "scale", scale);

        const bool hasNegativeScale = (scale.x() < 0.0 || scale.y() < 0.0 || scale.z() < 0.0);
        if(bakeTransform || hasNegativeScale){
            const Affine3 T = Eigen::Translation3d(translation) * q * Eigen::Scaling(scale);
            if(bakeTransform){
                bakeTransform = (*bakeTransform) * T.cast<float>();
            } else {
                bakeTransform = T.cast<float>();
            }
            group = new SgGroup;
            childContainer = group;
        } else {
            auto posTransform = new SgPosTransform;
            posTransform->setTranslation(translation);
            posTransform->setRotation(q);
            group = posTransform;
            childContainer = group;
            if(scale != Vector3::Ones()){
                auto scaleTransform = new SgScaleTransform;
                scaleTransform->setScale(scale);
                posTransform->addChild(scaleTransform);
                childContainer = scaleTransform;
            }
        }
    }

    string name;
    if(nodeInfo->read("name", name)){
        group->setName(name);
    }

    if(nodeInfo->find("skin")->isValid()){
        warnOnce(_("Warning: Skinning is not supported and skins are ignored."));
    }
    if(nodeInfo->find("camera")->isValid()){
        warnOnce(_("Warning: Cameras are ignored."));
    }

    int meshIndex;
    if(nodeInfo->read("mesh", meshIndex)){
        if(auto meshGroup = getOrCreateMesh(meshIndex)){
            childContainer->addChild(meshGroup);
        }
    }

    if(auto children = nodeInfo->findListing("children"); children->isValid()){
        for(auto& child : *children){
            if(auto childNode = convertNode(child->toInt(), depth + 1)){
                childContainer->addChild(childNode);
            }
        }
    }

    bakeTransform = prevBakeTransform;

    return group.retn();
}


SgGroup* GLTFSceneLoader::Impl::getOrCreateMesh(int meshIndex)
{
    /*
       While a reflection is being baked into the mesh vertices, the conversion
       result depends on the transform accumulated in the node instantiating the
       mesh, so the cache must be bypassed.
    */
    if(!bakeTransform){
        auto found = meshGroupMap.find(meshIndex);
        if(found != meshGroupMap.end()){
            return found->second;
        }
    }

    auto meshInfo = getListingElement(meshList, meshIndex, "meshes");

    SgGroupPtr meshGroup = new SgGroup;
    string name;
    if(meshInfo->read("name", name)){
        meshGroup->setName(name);
    }

    if(meshInfo->find("weights")->isValid()){
        warnOnce(_("Warning: Morph targets are not supported and the base geometries are used."));
    }

    if(auto primitiveList = meshInfo->findListing("primitives"); primitiveList->isValid()){
        for(int i = 0; i < primitiveList->size(); ++i){
            if(auto shape = convertPrimitive(primitiveList->at(i)->toMapping(), name, i)){
                meshGroup->addChild(shape);
            }
        }
    }

    if(bakeTransform){
        /*
           The vertex and normal arrays are created for each conversion and are not
           shared with the other instances here, so the accumulated transform can be
           baked into them directly. Line and point primitives only need the vertex
           positions to be baked.
        */
        for(int i = 0; i < meshGroup->numChildren(); ++i){
            auto child = meshGroup->child(i);
            if(auto shape = dynamic_cast<SgShape*>(child)){
                bakeTransformIntoMesh(shape->mesh(), *bakeTransform);
            } else if(auto plot = dynamic_cast<SgPlot*>(child)){
                if(plot->hasVertices()){
                    transformVertices(*plot->vertices(), *bakeTransform);
                    plot->invalidateBoundingBox();
                }
            }
        }
        return meshGroup.retn();
    }

    meshGroupMap[meshIndex] = meshGroup;
    return meshGroup;
}


bool GLTFSceneLoader::Impl::readPositionAttribute(Mapping* primitive, vector<float>& out_values)
{
    auto attributes = primitive->findMapping("attributes");
    if(!attributes->isValid()){
        return false;
    }
    int positionAccessor;
    if(!attributes->read("POSITION", positionAccessor)){
        warnOnce(_("Warning: A primitive without the POSITION attribute is skipped."));
        return false;
    }
    readAccessorAsFloat(positionAccessor, 3, out_values);
    return true;
}


SgNode* GLTFSceneLoader::Impl::convertPrimitive(
    Mapping* primitive, const string& meshName, int primitiveIndex)
{
    const int mode = primitive->get("mode", GLTF_TRIANGLES);
    if(mode == GLTF_POINTS || mode == GLTF_LINES ||
       mode == GLTF_LINE_LOOP || mode == GLTF_LINE_STRIP){
        return convertPlotPrimitive(primitive, mode, meshName, primitiveIndex);
    }
    if(mode != GLTF_TRIANGLES && mode != GLTF_TRIANGLE_STRIP && mode != GLTF_TRIANGLE_FAN){
        warnOnce(formatR(_("Warning: Primitive mode {0} is not supported "
                           "and the corresponding primitives are skipped."), mode));
        return nullptr;
    }

    auto attributes = primitive->findMapping("attributes");
    if(!attributes->isValid()){
        return nullptr;
    }

    SgShapePtr shape = new SgShape;
    auto mesh = shape->getOrCreateMesh();

    // Vertices
    vector<float> values;
    if(!readPositionAttribute(primitive, values)){
        return nullptr;
    }
    const int numVertices = values.size() / 3;
    auto vertices = mesh->getOrCreateVertices(numVertices);
    std::memcpy(vertices->data(), values.data(), sizeof(float) * values.size());

    // Triangle indices
    SgIndexArray indices;
    int indexAccessor;
    if(primitive->read("indices", indexAccessor)){
        readAccessorAsIndex(indexAccessor, indices);
    } else {
        indices.resize(numVertices);
        for(int i = 0; i < numVertices; ++i){
            indices[i] = i;
        }
    }
    convertTriangleIndices(mesh, indices, mode, numVertices);

    // Normals
    int normalAccessor;
    if(attributes->read("NORMAL", normalAccessor)){
        readAccessorAsFloat(normalAccessor, 3, values);
        if(values.size() / 3 == static_cast<size_t>(numVertices)){
            auto normals = mesh->getOrCreateNormals();
            normals->resize(numVertices);
            std::memcpy(normals->data(), values.data(), sizeof(float) * values.size());
        }
    } else {
        // Flat shading with the face normals, which the glTF specification defines
        // as the behavior when the normals are not given
        meshFilter.generateNormals(mesh, 0.0f);
    }

    // Texture coordinates
    int texCoordAccessor;
    if(attributes->read("TEXCOORD_0", texCoordAccessor)){
        readAccessorAsFloat(texCoordAccessor, 2, values);
        if(values.size() / 2 == static_cast<size_t>(numVertices)){
            auto texCoords = mesh->getOrCreateTexCoords();
            texCoords->resize(numVertices);
            for(int i = 0; i < numVertices; ++i){
                // The V components are inverted to convert the texture coordinate
                // origin from top-left (glTF) to bottom-left (Choreonoid)
                (*texCoords)[i] << values[i * 2], 1.0f - values[i * 2 + 1];
            }
        }
    }

    // Vertex colors
    int colorAccessor;
    if(attributes->read("COLOR_0", colorAccessor)){
        readAccessorAsFloat(colorAccessor, 0, values);
        const size_t stride = values.size() / numVertices;
        if(stride == 3 || stride == 4){
            auto colors = mesh->getOrCreateColors(numVertices);
            for(int i = 0; i < numVertices; ++i){
                (*colors)[i] << values[i * stride], values[i * stride + 1], values[i * stride + 2];
            }
        }
    }

    // Material
    const int materialIndex = primitive->get("material", -1);
    auto material = getOrCreateMaterial(materialIndex);
    shape->setMaterial(material);

    /*
       The base color texture is also set to the texture of the shape node so that
       renderers that do not support the PBR material extension can use the texture.
       The same texture instance is shared with the PBR material extension.
    */
    if(auto pbr = material->pbrExtension()){
        if(auto texture = pbr->baseColorTexture()){
            shape->setTexture(texture);
        }
    }

    bool doubleSided = false;
    if(materialIndex >= 0){
        getListingElement(materialList, materialIndex, "materials")->read("doubleSided", doubleSided);
    }
    mesh->setSolid(!doubleSided);

    if(!meshName.empty()){
        shape->setName(formatC("{0}_{1}", meshName, primitiveIndex));
    }

    return shape.retn();
}


SgNode* GLTFSceneLoader::Impl::convertPlotPrimitive(
    Mapping* primitive, int mode, const string& meshName, int primitiveIndex)
{
    vector<float> values;
    if(!readPositionAttribute(primitive, values)){
        return nullptr;
    }
    const int numVertices = values.size() / 3;

    SgIndexArray indices;
    int indexAccessor;
    if(primitive->read("indices", indexAccessor)){
        readAccessorAsIndex(indexAccessor, indices);
        for(auto index : indices){
            if(index < 0 || index >= numVertices){
                throw LoadingException(_("A primitive has an out-of-range vertex index."));
            }
        }
    } else {
        indices.resize(numVertices);
        for(int i = 0; i < numVertices; ++i){
            indices[i] = i;
        }
    }

    vector<float> colorValues;
    size_t colorStride = 0;
    int colorAccessor;
    if(auto attributes = primitive->findMapping("attributes");
       attributes->isValid() && attributes->read("COLOR_0", colorAccessor)){
        readAccessorAsFloat(colorAccessor, 0, colorValues);
        colorStride = colorValues.size() / numVertices;
        if(colorStride != 3 && colorStride != 4){
            colorValues.clear();
        }
    }

    SgPlotPtr plot;

    if(mode == GLTF_POINTS){
        auto pointSet = new SgPointSet;
        plot = pointSet;
        const int n = indices.size();
        auto vertices = pointSet->getOrCreateVertices(n);
        for(int i = 0; i < n; ++i){
            (*vertices)[i] = Eigen::Map<const Vector3f>(&values[indices[i] * 3]);
        }
        if(!colorValues.empty()){
            auto colors = pointSet->getOrCreateColors(n);
            for(int i = 0; i < n; ++i){
                (*colors)[i] = Eigen::Map<const Vector3f>(&colorValues[indices[i] * colorStride]);
            }
        }
    } else {
        auto lineSet = new SgLineSet;
        plot = lineSet;
        auto vertices = lineSet->getOrCreateVertices(numVertices);
        std::memcpy(vertices->data(), values.data(), sizeof(float) * values.size());

        auto& lineVertexIndices = lineSet->lineVertexIndices();
        const int n = indices.size();
        if(mode == GLTF_LINES){
            lineVertexIndices.assign(indices.begin(), indices.begin() + (n / 2) * 2);
        } else {
            lineVertexIndices.reserve(mode == GLTF_LINE_LOOP ? n * 2 : (n - 1) * 2);
            for(int i = 0; i < n - 1; ++i){
                lineVertexIndices.push_back(indices[i]);
                lineVertexIndices.push_back(indices[i + 1]);
            }
            if(mode == GLTF_LINE_LOOP && n >= 2){
                lineVertexIndices.push_back(indices[n - 1]);
                lineVertexIndices.push_back(indices[0]);
            }
        }
        if(!colorValues.empty()){
            auto colors = lineSet->getOrCreateColors(numVertices);
            for(int i = 0; i < numVertices; ++i){
                (*colors)[i] = Eigen::Map<const Vector3f>(&colorValues[i * colorStride]);
            }
            // The colors are indexed in the same way as the vertices
            lineSet->colorIndices() = lineVertexIndices;
        }
    }

    const int materialIndex = primitive->get("material", -1);
    if(materialIndex >= 0){
        plot->setMaterial(getOrCreateMaterial(materialIndex));
    }

    if(!meshName.empty()){
        plot->setName(formatC("{0}_{1}", meshName, primitiveIndex));
    }

    return plot.retn();
}


void GLTFSceneLoader::Impl::convertTriangleIndices(
    SgMesh* mesh, SgIndexArray& indices, int mode, int numVertices)
{
    for(auto index : indices){
        if(index < 0 || index >= numVertices){
            throw LoadingException(_("A primitive has an out-of-range vertex index."));
        }
    }

    auto& triangleVertices = mesh->triangleVertices();

    if(mode == GLTF_TRIANGLES){
        triangleVertices = std::move(indices);
        triangleVertices.resize(triangleVertices.size() / 3 * 3);

    } else if(mode == GLTF_TRIANGLE_STRIP){
        const int n = indices.size();
        triangleVertices.reserve(std::max(0, n - 2) * 3);
        for(int i = 0; i < n - 2; ++i){
            int v0, v1, v2;
            if(i % 2 == 0){
                v0 = indices[i]; v1 = indices[i + 1]; v2 = indices[i + 2];
            } else {
                v0 = indices[i + 1]; v1 = indices[i]; v2 = indices[i + 2];
            }
            if(v0 != v1 && v1 != v2 && v0 != v2){
                mesh->addTriangle(v0, v1, v2);
            }
        }
    } else if(mode == GLTF_TRIANGLE_FAN){
        const int n = indices.size();
        triangleVertices.reserve(std::max(0, n - 2) * 3);
        for(int i = 1; i < n - 1; ++i){
            mesh->addTriangle(indices[0], indices[i], indices[i + 1]);
        }
    }
}


SgMaterial* GLTFSceneLoader::Impl::getOrCreateMaterial(int materialIndex)
{
    if(materialIndex < 0){
        // The default material defined in the glTF specification
        if(!defaultMaterial){
            defaultMaterial = new SgMaterial;
            defaultMaterial->getOrCreatePbrExtension();
        }
        return defaultMaterial;
    }
    auto found = materialMap.find(materialIndex);
    if(found != materialMap.end()){
        return found->second;
    }
    auto material = convertMaterial(
        getListingElement(materialList, materialIndex, "materials"));
    materialMap[materialIndex] = material;
    return material;
}


SgMaterial* GLTFSceneLoader::Impl::convertMaterial(Mapping* materialInfo)
{
    SgMaterialPtr material = new SgMaterial;
    auto pbr = material->getOrCreatePbrExtension();

    string name;
    if(materialInfo->read("name", name)){
        material->setName(name);
    }

    string alphaMode = materialInfo->get("alphaMode", "OPAQUE");
    if(alphaMode == "MASK"){
        pbr->setAlphaMode(SgPbrMaterialExtension::Mask);
        pbr->setAlphaCutoff(materialInfo->get("alphaCutoff", 0.5));
    } else if(alphaMode == "BLEND"){
        pbr->setAlphaMode(SgPbrMaterialExtension::Blend);
    }

    auto readTextureRef = [this](Mapping* parent, const char* key, Mapping*& out_info) -> SgTexture* {
        auto info = parent->findMapping(key);
        if(!info->isValid()){
            return nullptr;
        }
        out_info = info;
        if(info->get("texCoord", 0) != 0){
            warnOnce(_("Warning: Only the texture coordinate set 0 (TEXCOORD_0) is supported."));
        }
        int index;
        if(!info->read("index", index)){
            return nullptr;
        }
        return getOrCreateTexture(index);
    };
    Mapping* textureRefInfo = nullptr;

    if(auto pmr = materialInfo->findMapping("pbrMetallicRoughness"); pmr->isValid()){
        if(auto baseColorFactor = pmr->findListing("baseColorFactor");
           baseColorFactor->isValid() && baseColorFactor->size() == 4){
            Vector3f color(
                baseColorFactor->at(0)->toFloat(),
                baseColorFactor->at(1)->toFloat(),
                baseColorFactor->at(2)->toFloat());
            pbr->setBaseColorFactor(color);
            /*
               The RGB components of the base color factor are also set to the diffuse color
               of SgMaterial, and its alpha component is mapped to the transparency, so that
               renderers that do not support the PBR material extension can approximate the
               material. Note that the values of the PBR material extension are the primary
               information when the extension exists.
            */
            material->setDiffuseColor(color);
            if(pbr->alphaMode() == SgPbrMaterialExtension::Blend){
                material->setTransparency(1.0f - baseColorFactor->at(3)->toFloat());
            }
        }
        pbr->setMetallicFactor(pmr->get("metallicFactor", 1.0));
        pbr->setRoughnessFactor(pmr->get("roughnessFactor", 1.0));

        if(auto texture = readTextureRef(pmr, "baseColorTexture", textureRefInfo)){
            pbr->setBaseColorTexture(texture);
        }
        if(auto texture = readTextureRef(pmr, "metallicRoughnessTexture", textureRefInfo)){
            pbr->setMetallicRoughnessTexture(texture);
        }
    }

    /*
       The classic specular parameters of SgMaterial are approximately reconstructed
       from the metallic-roughness values so that renderers that do not support the
       PBR material extension can reproduce the shininess of the material. The
       expression of the specular exponent is the inverse of the conversion done by
       GLTFSceneWriter. The specular color of a non-metal is modulated by the
       glossiness so that rough materials lose the specular reflection, which are
       perceptually almost invisible for the weak (4%) specular reflectance of
       non-metals. The specular color of a metal is the base color at the full
       strength without the modulation; the reflectance of a metal is the base color
       itself, and the roughness only spreads the reflection without extinguishing
       it. The spreading is expressed by the specular exponent alone. The Phong lobe
       with a small exponent keeps the peak strength and becomes broad, which
       approximates the blurred environment reflection of a rough metal well.
    */
    const float glossiness = 1.0f - pbr->roughnessFactor();
    const float metallic = pbr->metallicFactor();

    /*
       The dielectric (non-metal) specular color is 0.4 modulated by the glossiness
       by default. The base value 0.4 is ten times the physical specular reflectance
       (F0) of the standard non-metal, which is 4%, because the Phong specular term
       is not normalized and the physical value is almost invisible with it. The
       glossiness modulation expresses that the specular reflection of a rough
       non-metal is perceptually almost invisible.

       When the material has the KHR_materials_specular extension, which specifies
       the strength and the color of the specular reflection of a non-metal, the
       dielectric specular color is calculated from the extension values with the
       same boost factor, and the glossiness modulation is not applied because the
       extension explicitly specifies the strength. This expression is the exact
       inverse of the conversion done by GLTFSceneWriter, which stores the classic
       specular color as the specular color factor multiplied by 1/0.4, so that the
       original specular color of a classic material is exactly restored.
    */
    Vector3f dielectricSpecular = glossiness * Vector3f::Constant(0.4f);
    if(auto extensions = materialInfo->findMapping("extensions"); extensions->isValid()){
        if(auto ms = extensions->findMapping("KHR_materials_specular"); ms->isValid()){
            float specularFactor = ms->get("specularFactor", 1.0);
            Vector3f specularColorFactor(1.0f, 1.0f, 1.0f);
            read(ms, "specularColorFactor", specularColorFactor);
            dielectricSpecular =
                (0.4f * specularFactor * specularColorFactor).cwiseMin(1.0f);
            if(ms->findMapping("specularTexture")->isValid() ||
               ms->findMapping("specularColorTexture")->isValid()){
                warnOnce(_("Warning: The textures of the KHR_materials_specular "
                           "extension are not supported."));
            }
        }
    }

    /*
       The specular reflection of a metal substitutes for the environment reflection,
       which the Phong rendering does not have. A saturated base color such as the
       pure red has a structurally low perceived luminance (only 21% at the full
       strength for the pure red), and the specular reflection with such a color
       looks too dark although the same expression works well for grey metals like
       chrome. To compensate for it, the specular color of a metal is boosted so
       that its luminance reaches the target value. Grey and bright metals are not
       affected because the boost factor is floored at one, which also keeps the
       roundtrip of the classic materials converted by GLTFSceneWriter exact for
       them. The specular color components may exceed one by the boost, which is
       valid for the Phong shader; the output is just saturated.
    */
    Vector3f metalSpecular = pbr->baseColorFactor();
    const float metalLuminance =
        0.2126f * metalSpecular.x() + 0.7152f * metalSpecular.y() + 0.0722f * metalSpecular.z();
    constexpr float targetLuminance = 0.4f;
    constexpr float maxBoost = 3.0f;
    if(metalLuminance > 0.0f && metalLuminance < targetLuminance){
        metalSpecular *= std::min(maxBoost, targetLuminance / metalLuminance);
    }

    const Vector3f specularColor =
        (1.0f - metallic) * dielectricSpecular +
        metallic * metalSpecular;
    if(!specularColor.isZero()){
        material->setSpecularColor(specularColor);
        material->setSpecularExponent(std::max(1.0f, glossiness * 128.0f));
    }

    /*
       A metal hardly has the diffuse reflection and its appearance is dominated by
       the environment reflection tinted with the base color. Since the classic
       (Phong) rendering does not have the environment reflection, the diffuse color
       of a metallic material is replaced here with a small fraction of the (boosted)
       specular color so that a metal does not look like a bright matte surface while
       the surfaces off the specular reflection direction keep a dim tint of the
       reflection color instead of being completely black. Note that the residual
       diffuse color also determines the response to the ambient light because the
       ambient term of the renderer is the product of the ambient light intensity
       and the diffuse color; the ambient light in the scene serves as a rough
       substitute for the environment lighting of a metal through it. This
       adjustment is skipped when the metallic-roughness texture exists because the
       metallic factor is then just a multiplier of the texel values and the actual
       metallicness cannot be determined here.
    */
    if(metallic > 0.0f && !pbr->metallicRoughnessTexture()){
        material->setDiffuseColor(
            Vector3f(
                (1.0f - metallic) * material->diffuseColor() +
                metallic * 0.1f * metalSpecular));
    }

    if(auto texture = readTextureRef(materialInfo, "normalTexture", textureRefInfo)){
        pbr->setNormalTexture(texture);
        pbr->setNormalScale(textureRefInfo->get("scale", 1.0));
    }
    if(auto texture = readTextureRef(materialInfo, "occlusionTexture", textureRefInfo)){
        pbr->setOcclusionTexture(texture);
        pbr->setOcclusionStrength(textureRefInfo->get("strength", 1.0));
    }
    if(auto texture = readTextureRef(materialInfo, "emissiveTexture", textureRefInfo)){
        pbr->setEmissiveTexture(texture);
    }

    Vector3f emissiveFactor;
    if(read(materialInfo, "emissiveFactor", emissiveFactor)){
        pbr->setEmissiveFactor(emissiveFactor);
        /*
           The emissive color of SgMaterial is set only when the material does not have
           the emissive texture. In glTF, the actual emission is the product of the
           emissive factor and the emissive texture, so copying the factor alone to the
           classic emissive color would overestimate the emission for renderers that do
           not support the PBR material extension.
        */
        if(!pbr->emissiveTexture()){
            material->setEmissiveColor(emissiveFactor);
        }
    }

    if(auto extensions = materialInfo->findMapping("extensions"); extensions->isValid()){
        if(auto es = extensions->findMapping("KHR_materials_emissive_strength"); es->isValid()){
            pbr->setEmissiveStrength(es->get("emissiveStrength", 1.0));
        }
    }

    return material.retn();
}


SgTexture* GLTFSceneLoader::Impl::getOrCreateTexture(int textureIndex)
{
    auto found = textureMap.find(textureIndex);
    if(found != textureMap.end()){
        return found->second;
    }

    auto textureInfo = getListingElement(textureList, textureIndex, "textures");

    SgTexturePtr texture = new SgTexture;

    int imageIndex;
    if(textureInfo->read("source", imageIndex)){
        texture->setImage(getOrCreateImage(imageIndex));
    }

    int samplerIndex;
    if(textureInfo->read("sampler", samplerIndex)){
        auto samplerInfo = getListingElement(samplerList, samplerIndex, "samplers");
        const int wrapS = samplerInfo->get("wrapS", 0);
        const int wrapT = samplerInfo->get("wrapT", 0);
        texture->setRepeat(wrapS != GLTF_CLAMP_TO_EDGE, wrapT != GLTF_CLAMP_TO_EDGE);
    }

    if(auto extensions = textureInfo->findMapping("extensions"); extensions->isValid()){
        if(extensions->findMapping("KHR_texture_transform")->isValid()){
            warnOnce(_("Warning: The KHR_texture_transform extension is not supported."));
        }
    }

    textureMap[textureIndex] = texture;
    return texture;
}


SgImage* GLTFSceneLoader::Impl::getOrCreateImage(int imageIndex)
{
    auto found = imageMap.find(imageIndex);
    if(found != imageMap.end()){
        return found->second;
    }

    auto imageInfo = getListingElement(imageList, imageIndex, "images");

    SgImagePtr image = new SgImage;

    string uri;
    if(imageInfo->read("uri", uri)){
        if(uri.compare(0, 5, "data:") == 0){
            const auto pos = uri.find(";base64,");
            vector<uint8_t> data;
            if(pos == string::npos || !decodeBase64(uri, pos + 8, data)){
                os() << formatR(_("Warning: The data URI of image {0} cannot be decoded."),
                                imageIndex) << endl;
            } else if(!imageIO.load(image->image(), data.data(), data.size(), mout)){
                os() << formatR(_("Warning: The embedded image {0} cannot be loaded."),
                                imageIndex) << endl;
            } else {
                string mimeType = detectImageMimeType(data.data(), data.size());
                if(!mimeType.empty()){
                    image->setSourceData(new SgImage::SourceData(std::move(data), mimeType));
                }
            }
        } else {
            const string decodedUri = decodeUri(uri);
            filesystem::path imagePath(fromUTF8(decodedUri));
            if(imagePath.is_relative()){
                imagePath = directory / imagePath;
            }
            bool loaded = false;
            if(filesystem::exists(imagePath)){
                loaded = imageIO.load(image->image(), toUTF8(imagePath.string()), os());
                if(loaded){
                    image->setUriWithFilePathAndBaseDirectory(decodedUri, toUTF8(directory.string()));
                }
            } else {
                for(auto& searchDirectory : imageSearchDirectories){
                    auto fallbackPath = searchDirectory / fromUTF8(decodedUri);
                    if(filesystem::exists(fallbackPath)){
                        loaded = imageIO.load(image->image(), toUTF8(fallbackPath.string()), os());
                        if(loaded){
                            image->setUriWithFilePathAndBaseDirectory(
                                decodedUri, toUTF8(searchDirectory.string()));
                        }
                        break;
                    }
                }
            }
            if(!loaded){
                os() << formatR(_("Warning: Image file \"{0}\" cannot be loaded."), uri) << endl;
            }
        }
    } else {
        int bufferViewIndex;
        if(imageInfo->read("bufferView", bufferViewIndex)){
            auto bufferView = getListingElement(bufferViewList, bufferViewIndex, "bufferViews");
            const auto& data = getBufferData(bufferView->get("buffer", 0));
            const int offset = bufferView->get("byteOffset", 0);
            const int length = bufferView->get("byteLength", 0);
            if(offset < 0 || length <= 0 || static_cast<size_t>(offset) + length > data.size()){
                os() << formatR(_("Warning: Image {0} has an invalid buffer view."),
                                imageIndex) << endl;
            } else if(!imageIO.load(image->image(), data.data() + offset, length, mout)){
                os() << formatR(_("Warning: The embedded image {0} cannot be loaded."),
                                imageIndex) << endl;
            } else {
                string mimeType = detectImageMimeType(data.data() + offset, length);
                if(!mimeType.empty()){
                    image->setSourceData(
                        new SgImage::SourceData(
                            vector<uint8_t>(data.begin() + offset, data.begin() + offset + length),
                            mimeType));
                }
            }
        }
    }

    string name;
    if(imageInfo->read("name", name)){
        image->setName(name);
    }

    imageMap[imageIndex] = image;
    return image;
}


const vector<uint8_t>& GLTFSceneLoader::Impl::getBufferData(int bufferIndex)
{
    auto found = bufferDataMap.find(bufferIndex);
    if(found != bufferDataMap.end()){
        return found->second;
    }

    auto bufferInfo = getListingElement(bufferList, bufferIndex, "buffers");
    const int byteLength = bufferInfo->get("byteLength", 0);

    vector<uint8_t> data;
    string uri;
    if(!bufferInfo->read("uri", uri)){
        // A buffer without the uri refers to the GLB-stored binary chunk
        if(!hasGlbBinChunk){
            throw LoadingException(
                formatR(_("Buffer {0} does not have the data source."), bufferIndex));
        }
        data = std::move(glbBinChunk);
        hasGlbBinChunk = false;
    } else if(uri.compare(0, 5, "data:") == 0){
        const auto pos = uri.find(";base64,");
        if(pos == string::npos || !decodeBase64(uri, pos + 8, data)){
            throw LoadingException(
                formatR(_("The data URI of buffer {0} cannot be decoded."), bufferIndex));
        }
    } else {
        filesystem::path bufferPath(fromUTF8(decodeUri(uri)));
        if(bufferPath.is_relative()){
            bufferPath = directory / bufferPath;
        }
        ifstream file(bufferPath, ios::binary | ios::ate);
        if(!file){
            throw LoadingException(
                formatR(_("Buffer file \"{0}\" cannot be opened."), uri));
        }
        const auto size = file.tellg();
        file.seekg(0);
        data.resize(size);
        file.read(reinterpret_cast<char*>(data.data()), size);
        if(!file){
            throw LoadingException(
                formatR(_("Buffer file \"{0}\" cannot be read."), uri));
        }
    }

    if(data.size() < static_cast<size_t>(byteLength)){
        throw LoadingException(
            formatR(_("The data of buffer {0} is smaller than the specified byte length."),
                    bufferIndex));
    }

    return bufferDataMap.emplace(bufferIndex, std::move(data)).first->second;
}


Mapping* GLTFSceneLoader::Impl::getListingElement(
    Listing* listing, int index, const char* listingName)
{
    if(!listing || !listing->isValid() || index < 0 || index >= listing->size()){
        throw LoadingException(
            formatR(_("The element {0} of \"{1}\" is not found."), index, listingName));
    }
    return listing->at(index)->toMapping();
}


void GLTFSceneLoader::Impl::getAccessorDataPointer
(Mapping* accessor, int numComponents,
 const uint8_t*& out_pointer, int& out_stride, int& out_count, int& out_componentType,
 bool& out_normalized)
{
    out_count = accessor->get<int>("count");
    out_componentType = accessor->get<int>("componentType");
    out_normalized = accessor->get("normalized", false);

    const int componentSize = componentSizeOf(out_componentType);
    if(componentSize == 0){
        throw LoadingException(
            formatR(_("Component type {0} of an accessor is invalid."), out_componentType));
    }
    const int elementSize = componentSize * numComponents;

    if(accessor->find("sparse")->isValid()){
        warnOnce(_("Warning: Sparse accessors are not supported and their base data are used."));
    }

    int bufferViewIndex;
    if(!accessor->read("bufferView", bufferViewIndex)){
        // An accessor without the buffer view means zero-filled data
        out_pointer = nullptr;
        out_stride = 0;
        return;
    }
    auto bufferView = getListingElement(bufferViewList, bufferViewIndex, "bufferViews");
    const auto& data = getBufferData(bufferView->get("buffer", 0));
    const int bufferViewOffset = bufferView->get("byteOffset", 0);
    const int bufferViewLength = bufferView->get("byteLength", 0);
    const int accessorOffset = accessor->get("byteOffset", 0);
    const int stride = bufferView->get("byteStride", 0);
    out_stride = (stride > 0) ? stride : elementSize;

    // Check if the accessed range is within the buffer and the buffer view
    const size_t end = static_cast<size_t>(bufferViewOffset) + accessorOffset +
        (out_count > 0 ? (static_cast<size_t>(out_count) - 1) * out_stride + elementSize : 0);
    if(end > data.size() ||
       end > static_cast<size_t>(bufferViewOffset) + bufferViewLength){
        throw LoadingException(_("An accessor accesses the outside of the buffer."));
    }

    out_pointer = data.data() + bufferViewOffset + accessorOffset;
}


void GLTFSceneLoader::Impl::readAccessorAsFloat
(int accessorIndex, int expectedNumComponents, vector<float>& out)
{
    auto accessor = getListingElement(accessorList, accessorIndex, "accessors");

    const int numComponents = numComponentsOf(accessor->get<string>("type"));
    if(numComponents == 0 ||
       (expectedNumComponents > 0 && numComponents != expectedNumComponents)){
        throw LoadingException(_("An accessor has an unexpected element type."));
    }

    const uint8_t* pointer;
    int stride, count, componentType;
    bool normalized;
    getAccessorDataPointer(accessor, numComponents, pointer, stride, count, componentType, normalized);

    out.resize(static_cast<size_t>(count) * numComponents);
    if(!pointer){
        std::fill(out.begin(), out.end(), 0.0f);
        return;
    }

    for(int i = 0; i < count; ++i){
        const uint8_t* p = pointer + static_cast<size_t>(i) * stride;
        float* q = &out[static_cast<size_t>(i) * numComponents];
        switch(componentType){
        case GLTF_FLOAT:
            std::memcpy(q, p, sizeof(float) * numComponents);
            break;
        case GLTF_BYTE:
            for(int j = 0; j < numComponents; ++j){
                const auto v = static_cast<int8_t>(p[j]);
                q[j] = normalized ? std::max(v / 127.0f, -1.0f) : static_cast<float>(v);
            }
            break;
        case GLTF_UNSIGNED_BYTE:
            for(int j = 0; j < numComponents; ++j){
                q[j] = normalized ? p[j] / 255.0f : static_cast<float>(p[j]);
            }
            break;
        case GLTF_SHORT:
            for(int j = 0; j < numComponents; ++j){
                int16_t v;
                std::memcpy(&v, p + j * 2, 2);
                q[j] = normalized ? std::max(v / 32767.0f, -1.0f) : static_cast<float>(v);
            }
            break;
        case GLTF_UNSIGNED_SHORT:
            for(int j = 0; j < numComponents; ++j){
                uint16_t v;
                std::memcpy(&v, p + j * 2, 2);
                q[j] = normalized ? v / 65535.0f : static_cast<float>(v);
            }
            break;
        case GLTF_UNSIGNED_INT:
            for(int j = 0; j < numComponents; ++j){
                uint32_t v;
                std::memcpy(&v, p + j * 4, 4);
                q[j] = static_cast<float>(v);
            }
            break;
        }
    }
}


void GLTFSceneLoader::Impl::readAccessorAsIndex(int accessorIndex, SgIndexArray& out)
{
    auto accessor = getListingElement(accessorList, accessorIndex, "accessors");

    if(accessor->get<string>("type") != "SCALAR"){
        throw LoadingException(_("An index accessor must have the SCALAR type."));
    }

    const uint8_t* pointer;
    int stride, count, componentType;
    bool normalized;
    getAccessorDataPointer(accessor, 1, pointer, stride, count, componentType, normalized);

    out.clear();
    out.reserve(count);
    if(!pointer){
        out.resize(count, 0);
        return;
    }

    for(int i = 0; i < count; ++i){
        const uint8_t* p = pointer + static_cast<size_t>(i) * stride;
        switch(componentType){
        case GLTF_UNSIGNED_BYTE:
            out.push_back(*p);
            break;
        case GLTF_UNSIGNED_SHORT: {
            uint16_t v;
            std::memcpy(&v, p, 2);
            out.push_back(v);
            break;
        }
        case GLTF_UNSIGNED_INT: {
            uint32_t v;
            std::memcpy(&v, p, 4);
            out.push_back(static_cast<int>(v));
            break;
        }
        default:
            throw LoadingException(
                formatR(_("Component type {0} is invalid for an index accessor."), componentType));
        }
    }
}
