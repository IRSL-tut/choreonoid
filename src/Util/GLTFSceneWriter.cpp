#include "GLTFSceneWriter.h"
#include "SceneDrawables.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "NullOut.h"
#include "UTF8.h"
#include "Format.h"
#include <stb/stb_image_write.h>
#include <fstream>
#include <filesystem>
#include <map>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace filesystem = std::filesystem;

namespace {

// glTF 2.0 constants
constexpr int GLTF_UNSIGNED_SHORT = 5123;
constexpr int GLTF_UNSIGNED_INT = 5125;
constexpr int GLTF_FLOAT = 5126;
constexpr int GLTF_POINTS = 0;
constexpr int GLTF_LINES = 1;
constexpr int GLTF_TRIANGLES = 4;
constexpr int GLTF_ARRAY_BUFFER = 34962;
constexpr int GLTF_ELEMENT_ARRAY_BUFFER = 34963;
constexpr int GLTF_REPEAT = 10497;
constexpr int GLTF_CLAMP_TO_EDGE = 33071;

constexpr uint32_t GLB_MAGIC = 0x46546C67; // "glTF"
constexpr uint32_t GLB_CHUNK_JSON = 0x4E4F534A; // "JSON"
constexpr uint32_t GLB_CHUNK_BIN = 0x004E4942; // "BIN"

/**
   A minimal writer to output a JSON text, which is needed because the glTF
   specification requires strict JSON and the flow style of YAMLWriter does
   not exactly conform to it.
*/
class JsonWriter
{
public:
    string text;

    void startObject() { prepareForValue(); text += '{'; isFirstStack.push_back(true); }
    void endObject() { text += '}'; isFirstStack.pop_back(); }
    void startArray() { prepareForValue(); text += '['; isFirstStack.push_back(true); }
    void endArray() { text += ']'; isFirstStack.pop_back(); }

    void key(const char* name)
    {
        separate();
        putString(name);
        text += ':';
        hasPendingKey = true;
    }

    void put(const string& value) { prepareForValue(); putString(value.c_str()); }
    void put(const char* value) { prepareForValue(); putString(value); }
    void put(bool value) { prepareForValue(); text += value ? "true" : "false"; }
    void put(int value) { prepareForValue(); text += formatC("{}", value); }
    void put(size_t value) { prepareForValue(); text += formatC("{}", value); }

    void put(double value)
    {
        prepareForValue();
        if(!std::isfinite(value)){
            value = 0.0; // JSON does not have inf / nan
        }
        text += formatC("{}", value);
    }

    template<class ValueType> void keyValue(const char* name, const ValueType& value)
    {
        key(name);
        put(value);
    }

private:
    vector<bool> isFirstStack{ true };
    bool hasPendingKey = false;

    void separate()
    {
        if(!isFirstStack.back()){
            text += ',';
        }
        isFirstStack.back() = false;
    }

    void prepareForValue()
    {
        if(hasPendingKey){
            hasPendingKey = false;
        } else {
            separate();
        }
    }

    void putString(const char* value)
    {
        text += '"';
        for(const char* p = value; *p; ++p){
            const unsigned char c = *p;
            switch(c){
            case '"': text += "\\\""; break;
            case '\\': text += "\\\\"; break;
            case '\n': text += "\\n"; break;
            case '\r': text += "\\r"; break;
            case '\t': text += "\\t"; break;
            default:
                if(c < 0x20){
                    text += formatC("\\u{:04x}", static_cast<int>(c));
                } else {
                    text += static_cast<char>(c);
                }
                break;
            }
        }
        text += '"';
    }
};

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

/*
   Whether the material is intended to represent a metal is estimated from
   the classic parameters because the classic material model does not have
   the metallic information. The following patterns, which are the classic
   idioms for expressing metals, are regarded as metals.

   1. The specular color is clearly saturated (tinted). The specular
      reflection of a non-metal is physically white, so a tinted specular
      color expresses the colored reflection unique to metals such as gold
      and copper.
   2. The specular color is strong and the diffuse color is darker than it.
      Grey metals such as chrome and silver are classically expressed by a
      dark diffuse color with a strong highlight because a metal hardly has
      the diffuse reflection.

   In both patterns, the specular color is strictly brighter than the diffuse
   color because a metal hardly has the diffuse reflection and the reflection
   dominates the metallic appearance; the classic idioms express it with a
   darkened diffuse color. This criterion excludes a plastic material whose
   specular color is just a dimmed or exact copy of the diffuse color, which
   is a common lazy idiom of classic authoring tools and CAD exporters and
   would otherwise be misclassified by the pattern 1. A bright diffuse color
   with a moderate grey specular color, which is a typical shiny plastic,
   matches neither of the patterns. The brightness of a tinted specular color
   is evaluated with the maximum component instead of the luminance because
   the luminance structurally underestimates saturated colors; the luminance
   of the pure red is only 21% even at the full strength. Note that the
   detection is inherently ambiguous for dark glossy plastics and dull metals
   because they have almost the same forms in the classic parameters; the
   thresholds are chosen conservatively so that plastic-like materials are
   not misclassified as metals.
*/
bool isMetalLikeClassicMaterial(const SgMaterial* material)
{
    const Vector3f& diffuse = material->diffuseColor();
    const Vector3f& specular = material->specularColor();
    auto luminance = [](const Vector3f& c){
        return 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z();
    };
    const float specularLuminance = luminance(specular);
    const float diffuseLuminance = luminance(diffuse);
    const float maxSpecularComponent = specular.maxCoeff();
    const float specularSaturation =
        (maxSpecularComponent > 0.0f) ?
        (maxSpecularComponent - specular.minCoeff()) / maxSpecularComponent : 0.0f;
    const bool isColoredMetal =
        specularSaturation >= 0.3f && maxSpecularComponent >= 0.3f &&
        maxSpecularComponent > diffuse.maxCoeff();
    const bool isGreyMetal =
        specularLuminance >= 0.5f && specularLuminance > diffuseLuminance + 0.2f;
    return isColoredMetal || isGreyMetal;
}

// Percent-encode the characters that are not allowed in URIs.
// The slashes are kept as they are the path segment separators.
string encodeUri(const string& text)
{
    static const char* hex = "0123456789ABCDEF";
    string encoded;
    encoded.reserve(text.size());
    for(const char c : text){
        const auto b = static_cast<unsigned char>(c);
        if(isalnum(b) || b == '-' || b == '_' || b == '.' || b == '~' || b == '/'){
            encoded += c;
        } else {
            encoded += '%';
            encoded += hex[b >> 4];
            encoded += hex[b & 0xf];
        }
    }
    return encoded;
}

}

namespace cnoid {

class GLTFSceneWriter::Impl
{
public:
    GLTFSceneWriter* self;
    PolymorphicSceneNodeFunctionSet writeFunctions;
    int convertedNodeIndex;
    std::ostream& os();

    // Elements of the glTF document being constructed
    vector<uint8_t> binary; // the content of the single buffer
    struct BufferViewDef { size_t offset; size_t length; int target; };
    vector<BufferViewDef> bufferViews;
    struct AccessorDef {
        int bufferView; int componentType; int count; const char* type;
        bool hasMinMax; Vector3f minValues; Vector3f maxValues;
    };
    vector<AccessorDef> accessors;
    struct PrimitiveDef {
        int position = -1; int normal = -1; int texCoord = -1; int color = -1;
        int indices = -1; int material = -1;
        int mode = GLTF_TRIANGLES;
    };
    struct MeshDef { string name; vector<PrimitiveDef> primitives; };
    vector<MeshDef> meshes;
    map<pair<SgObject*, int>, int> meshMap; // (mesh or plot, material index) -> mesh index
    struct NodeDef {
        string name; int mesh = -1; vector<int> children;
        bool hasMatrix = false; Affine3 M;
        bool hasTranslation = false; Vector3 translation;
        bool hasRotation = false; Quaterniond rotation;
        bool hasScale = false; Vector3 scale;
    };
    vector<NodeDef> nodes;
    struct MaterialDef {
        SgMaterialPtr material;
        bool doubleSided;
        bool isInferredMetal = false;
        // The texture indices are resolved when the material is registered so that
        // all the image data are stored in the buffer before the JSON output
        int baseColorTexture = -1;
        int metallicRoughnessTexture = -1;
        int normalTexture = -1;
        int occlusionTexture = -1;
        int emissiveTexture = -1;
    };
    vector<MaterialDef> materials;
    map<tuple<SgMaterial*, SgTexture*, bool>, int> materialMap;
    struct SamplerDef { bool repeatS; bool repeatT; };
    vector<SamplerDef> samplers;
    map<pair<bool, bool>, int> samplerMap;
    struct TextureDef { int source; int sampler; };
    vector<TextureDef> textures;
    unordered_map<SgTexture*, int> textureMap;
    struct ImageDef { int bufferView = -1; string uri; string name; string mimeType; };
    vector<ImageDef> images;
    unordered_map<SgImage*, int> imageMap;
    SgMaterialPtr defaultMaterial;
    bool useEmissiveStrength;
    bool useSpecularExtension;
    bool isMaterialEnabled;
    bool useExternalImages;
    string outputImageDirectory;
    filesystem::path imageOutputDirPath;

    Impl(GLTFSceneWriter* self);
    void clearDocument();
    bool writeScene(const string& filename, SgNode* node);
    int convertNode(SgNode* node);
    void setNameAndChildren(NodeDef& def, SgNode* node, SgGroup* group);
    int getOrCreateMeshIndex(SgShape* shape);
    int getOrCreatePlotMeshIndex(SgPlot* plot, int mode);
    void convertPrimitive(SgMesh* mesh, int materialIndex, PrimitiveDef& primitive);
    void convertPlotPrimitive(SgPlot* plot, int mode, int materialIndex, PrimitiveDef& primitive);
    int getOrCreateMaterialIndex(SgMaterial* material, SgTexture* shapeTexture, bool doubleSided);
    int getOrCreateTextureIndex(SgTexture* texture);
    int getOrCreateImageIndex(SgImage* image);
    int getOrCreateSamplerIndex(bool repeatS, bool repeatT);
    int addBufferView(const void* data, size_t size, int target);
    int addFloatAccessor(const vector<float>& values, int numComponentsPerElement,
                         const char* type, bool withMinMax);
    int addIndexAccessor(const SgIndexArray& indices, int numVertices);
    string emitJson(const string& binUri, size_t binSize);
    void emitMaterial(JsonWriter& json, const MaterialDef& def);
    void emitTextureRef(JsonWriter& json, const char* key, int index,
                        const char* scaleKey = nullptr, double scale = 1.0);
};

}


GLTFSceneWriter::GLTFSceneWriter()
{
    impl = new Impl(this);
}


GLTFSceneWriter::Impl::Impl(GLTFSceneWriter* self)
    : self(self)
{
    isMaterialEnabled = true;
    useExternalImages = false;

    writeFunctions.setFunction<SgShape>(
        [this](SgShape* shape){
            NodeDef def;
            def.mesh = getOrCreateMeshIndex(shape);
            setNameAndChildren(def, shape, nullptr);
        });
    writeFunctions.setFunction<SgPosTransform>(
        [this](SgPosTransform* transform){
            NodeDef def;
            const auto& T = transform->T();
            if(!T.translation().isZero()){
                def.hasTranslation = true;
                def.translation = T.translation();
            }
            Quaterniond q(T.linear());
            if(std::abs(q.w()) < 1.0 - 1.0e-12){
                def.hasRotation = true;
                def.rotation = q.normalized();
            }
            setNameAndChildren(def, transform, transform);
        });
    writeFunctions.setFunction<SgScaleTransform>(
        [this](SgScaleTransform* transform){
            NodeDef def;
            if(transform->scaleOrientation().isApprox(Matrix3::Identity())){
                def.hasScale = true;
                def.scale = transform->scale();
            } else {
                def.hasMatrix = true;
                transform->getTransform(def.M);
            }
            setNameAndChildren(def, transform, transform);
        });
    writeFunctions.setFunction<SgTransform>(
        [this](SgTransform* transform){
            // The generic case including SgAffineTransform
            NodeDef def;
            def.hasMatrix = true;
            transform->getTransform(def.M);
            setNameAndChildren(def, transform, transform);
        });
    writeFunctions.setFunction<SgGroup>(
        [this](SgGroup* group){
            NodeDef def;
            setNameAndChildren(def, group, group);
        });
    writeFunctions.setFunction<SgSwitchableGroup>(
        [this](SgSwitchableGroup* group){
            if(group->isTurnedOn()){
                NodeDef def;
                setNameAndChildren(def, group, group);
            }
        });
    writeFunctions.setFunction<SgLineSet>(
        [this](SgLineSet* lineSet){
            NodeDef def;
            def.mesh = getOrCreatePlotMeshIndex(lineSet, GLTF_LINES);
            setNameAndChildren(def, lineSet, nullptr);
        });
    writeFunctions.setFunction<SgPointSet>(
        [this](SgPointSet* pointSet){
            NodeDef def;
            def.mesh = getOrCreatePlotMeshIndex(pointSet, GLTF_POINTS);
            setNameAndChildren(def, pointSet, nullptr);
        });
    writeFunctions.setFunction<SgNode>(
        [this](SgNode* node){
            os() << formatR(_("Warning: {0} node \"{1}\" is not supported and skipped."),
                            node->className(), node->name()) << endl;
        });
    writeFunctions.updateDispatchTable();
}


GLTFSceneWriter::~GLTFSceneWriter()
{
    delete impl;
}


void GLTFSceneWriter::setMessageSink(std::ostream& os)
{
    AbstractSceneWriter::setMessageSink(os);
}


void GLTFSceneWriter::setMaterialEnabled(bool on)
{
    impl->isMaterialEnabled = on;
}


bool GLTFSceneWriter::isMaterialEnabled() const
{
    return impl->isMaterialEnabled;
}


bool GLTFSceneWriter::writeScene(const std::string& filename, SgNode* node)
{
    return impl->writeScene(filename, node);
}


void GLTFSceneWriter::Impl::clearDocument()
{
    binary.clear();
    bufferViews.clear();
    accessors.clear();
    meshes.clear();
    meshMap.clear();
    nodes.clear();
    materials.clear();
    materialMap.clear();
    samplers.clear();
    samplerMap.clear();
    textures.clear();
    textureMap.clear();
    images.clear();
    imageMap.clear();
    defaultMaterial.reset();
    useEmissiveStrength = false;
    useSpecularExtension = false;
}


std::ostream& GLTFSceneWriter::Impl::os()
{
    return self->os();
}


bool GLTFSceneWriter::Impl::writeScene(const string& filename, SgNode* node)
{
    clearDocument();

    filesystem::path filepath(fromUTF8(filename));
    string ext = filepath.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    const bool isGlb = (ext == ".glb");
    if(!isGlb && ext != ".gltf"){
        os() << formatR(_("The extension of the output file \"{0}\" must be .glb or .gltf."),
                        filename) << endl;
        return false;
    }

    /*
       In the .gltf format, the texture images are output as external image files
       referenced by URIs following the common glTF convention, while the images
       are embedded in the binary data in the GLB format. The information of the
       output image files is kept over multiple writeScene calls so that an image
       file can be shared by the glTF files output into the same directory. The
       information is cleared when the output directory changes because the files
       in the previous directory cannot be referred to by the relative URIs.
    */
    useExternalImages = !isGlb;
    auto outputDirPath = filepath.parent_path();
    if(outputDirPath != imageOutputDirPath){
        self->clearImageFileInformation();
        imageOutputDirPath = outputDirPath;
    }
    outputImageDirectory = toUTF8(outputDirPath.string());

    /*
       The texture images are stored in the top-down order in glTF, while the
       bottom-up order is used in Choreonoid. The images are flipped in encoding
       them, and the V components of the texture coordinates are also inverted
       in the primitive conversion.
    */
    stbi_flip_vertically_on_write(1);

    const int sceneRootIndex = convertNode(node);
    if(sceneRootIndex < 0){
        os() << formatR(_("The scene of \"{0}\" has no exportable contents."), filename) << endl;
        return false;
    }

    /*
       The coordinate values are output as they are although the glTF specification
       defines the Y-up coordinate system, so that the mesh data keep the same
       coordinates as in Choreonoid. This is symmetric with GLTFSceneLoader, which
       loads the coordinate values without the conversion by default.
    */

    bool result = false;

    try {
        if(isGlb){
            string jsonText = emitJson("", binary.size());
            ofstream file(filepath, ios::binary | ios::trunc);
            if(!file){
                throw std::runtime_error(strerror(errno));
            }
            while(jsonText.size() % 4 != 0){
                jsonText += ' ';
            }
            while(binary.size() % 4 != 0){
                binary.push_back(0);
            }
            uint32_t header[3];
            header[0] = GLB_MAGIC;
            header[1] = 2;
            header[2] = 12 + 8 + jsonText.size() + (binary.empty() ? 0 : 8 + binary.size());
            file.write(reinterpret_cast<char*>(header), sizeof(header));
            uint32_t chunkHeader[2];
            chunkHeader[0] = jsonText.size();
            chunkHeader[1] = GLB_CHUNK_JSON;
            file.write(reinterpret_cast<char*>(chunkHeader), sizeof(chunkHeader));
            file.write(jsonText.data(), jsonText.size());
            if(!binary.empty()){
                chunkHeader[0] = binary.size();
                chunkHeader[1] = GLB_CHUNK_BIN;
                file.write(reinterpret_cast<char*>(chunkHeader), sizeof(chunkHeader));
                file.write(reinterpret_cast<const char*>(binary.data()), binary.size());
            }
            if(!file){
                throw std::runtime_error(strerror(errno));
            }
        } else {
            auto binPath = filepath;
            binPath.replace_extension(".bin");
            string binUri;
            if(!binary.empty()){
                binUri = encodeUri(toUTF8(binPath.filename().string()));
                ofstream binFile(binPath, ios::binary | ios::trunc);
                binFile.write(reinterpret_cast<const char*>(binary.data()), binary.size());
                if(!binFile){
                    throw std::runtime_error(strerror(errno));
                }
            }
            string jsonText = emitJson(binUri, binary.size());
            ofstream file(filepath, ios::binary | ios::trunc);
            file.write(jsonText.data(), jsonText.size());
            if(!file){
                throw std::runtime_error(strerror(errno));
            }
        }
        result = true;
    }
    catch(const std::exception& ex){
        os() << formatR(_("glTF file \"{0}\" cannot be written. {1}"), filename, ex.what()) << endl;
    }

    clearDocument();

    return result;
}


int GLTFSceneWriter::Impl::convertNode(SgNode* node)
{
    convertedNodeIndex = -1;
    writeFunctions.dispatch(node);
    return convertedNodeIndex;
}


void GLTFSceneWriter::Impl::setNameAndChildren(NodeDef& def, SgNode* node, SgGroup* group)
{
    if(!node->name().empty()){
        def.name = node->name();
    }
    if(group){
        for(int i = 0; i < group->numChildren(); ++i){
            const int childIndex = convertNode(group->child(i));
            if(childIndex >= 0){
                def.children.push_back(childIndex);
            }
        }
    }
    if(def.mesh < 0 && def.children.empty()){
        convertedNodeIndex = -1; // omit an empty node
        return;
    }
    nodes.push_back(std::move(def));
    convertedNodeIndex = nodes.size() - 1;
}


int GLTFSceneWriter::Impl::getOrCreateMeshIndex(SgShape* shape)
{
    auto mesh = shape->mesh();
    if(!mesh || !mesh->hasVertices() || mesh->numTriangles() == 0){
        return -1;
    }
    int materialIndex = -1;
    if(isMaterialEnabled){
        auto material = shape->material();
        SgTexture* shapeTexture =
            (material && material->hasPbrExtension()) ? nullptr : shape->texture();
        const bool doubleSided = !mesh->isSolid();
        materialIndex = getOrCreateMaterialIndex(material, shapeTexture, doubleSided);
    }

    const auto key = make_pair(static_cast<SgObject*>(mesh), materialIndex);
    auto found = meshMap.find(key);
    if(found != meshMap.end()){
        return found->second;
    }

    MeshDef def;
    if(!mesh->name().empty()){
        def.name = mesh->name();
    } else if(!shape->name().empty()){
        def.name = shape->name();
    }
    def.primitives.emplace_back();
    convertPrimitive(mesh, materialIndex, def.primitives.back());
    meshes.push_back(std::move(def));

    const int index = meshes.size() - 1;
    meshMap[key] = index;
    return index;
}


int GLTFSceneWriter::Impl::getOrCreatePlotMeshIndex(SgPlot* plot, int mode)
{
    if(!plot->hasVertices()){
        return -1;
    }
    const int materialIndex =
        isMaterialEnabled ? getOrCreateMaterialIndex(plot->material(), nullptr, false) : -1;

    const auto key = make_pair(static_cast<SgObject*>(plot), materialIndex);
    auto found = meshMap.find(key);
    if(found != meshMap.end()){
        return found->second;
    }

    MeshDef def;
    if(!plot->name().empty()){
        def.name = plot->name();
    }
    def.primitives.emplace_back();
    convertPlotPrimitive(plot, mode, materialIndex, def.primitives.back());
    meshes.push_back(std::move(def));

    const int index = meshes.size() - 1;
    meshMap[key] = index;
    return index;
}


void GLTFSceneWriter::Impl::convertPlotPrimitive
(SgPlot* plot, int mode, int materialIndex, PrimitiveDef& primitive)
{
    primitive.mode = mode;
    primitive.material = materialIndex;

    const auto& orgVertices = *plot->vertices();
    const bool hasColors = plot->hasColors();
    const auto& colorIndices = plot->colorIndices();

    vector<float> vertices;
    vector<float> colors;

    SgIndexArray indices;
    if(mode == GLTF_LINES){
        indices = static_cast<SgLineSet*>(plot)->lineVertexIndices();
    }

    if(indices.empty() || colorIndices.empty()){
        // The vertices (and the vertex-wise colors if any) can be output as they are
        const int n = orgVertices.size();
        vertices.reserve(n * 3);
        for(int i = 0; i < n; ++i){
            const auto& v = orgVertices[i];
            vertices.insert(vertices.end(), { v.x(), v.y(), v.z() });
        }
        if(hasColors){
            const auto& orgColors = *plot->colors();
            const int numColors = orgColors.size();
            colors.reserve(n * 3);
            for(int i = 0; i < n; ++i){
                const auto& c = orgColors[std::min(i, numColors - 1)];
                colors.insert(colors.end(), { c.x(), c.y(), c.z() });
            }
        }
    } else {
        // The colors are indexed separately and the vertices are expanded to
        // realize the unified indexing of glTF
        const auto& orgColors = *plot->colors();
        map<pair<int, int>, int> vertexMap;
        SgIndexArray unifiedIndices;
        unifiedIndices.reserve(indices.size());
        for(size_t i = 0; i < indices.size(); ++i){
            const int vi = indices[i];
            const int ci = colorIndices[i];
            auto inserted = vertexMap.emplace(make_pair(vi, ci), vertexMap.size());
            if(inserted.second){
                const auto& v = orgVertices[vi];
                vertices.insert(vertices.end(), { v.x(), v.y(), v.z() });
                const auto& c = orgColors[ci];
                colors.insert(colors.end(), { c.x(), c.y(), c.z() });
            }
            unifiedIndices.push_back(inserted.first->second);
        }
        indices = std::move(unifiedIndices);
    }

    const int numVertices = vertices.size() / 3;
    primitive.position = addFloatAccessor(vertices, 3, "VEC3", true);
    if(!colors.empty()){
        primitive.color = addFloatAccessor(colors, 3, "VEC3", false);
    }
    if(!indices.empty()){
        primitive.indices = addIndexAccessor(indices, numVertices);
    }
}


void GLTFSceneWriter::Impl::convertPrimitive(SgMesh* mesh, int materialIndex, PrimitiveDef& primitive)
{
    primitive.material = materialIndex;

    const auto& orgTriangleVertices = mesh->triangleVertices();
    const auto& orgVertices = *mesh->vertices();
    const bool hasNormals = mesh->hasNormals();
    const bool hasTexCoords = mesh->hasTexCoords();
    const bool hasColors = mesh->hasColors();

    const auto& normalIndices = mesh->normalIndices();
    const auto& texCoordIndices = mesh->texCoordIndices();
    const auto& colorIndices = mesh->colorIndices();
    const bool hasSeparateIndices =
        (hasNormals && !normalIndices.empty()) ||
        (hasTexCoords && !texCoordIndices.empty()) ||
        (hasColors && !colorIndices.empty());

    vector<float> vertices;
    vector<float> normals;
    vector<float> texCoords;
    vector<float> colors;
    SgIndexArray triangleVertices;

    auto pushVertex = [&](int vertexIndex, int normalIndex, int texCoordIndex, int colorIndex){
        const auto& v = orgVertices[vertexIndex];
        vertices.insert(vertices.end(), { v.x(), v.y(), v.z() });
        if(hasNormals){
            const auto& n = (*mesh->normals())[normalIndex];
            normals.insert(normals.end(), { n.x(), n.y(), n.z() });
        }
        if(hasTexCoords){
            const auto& t = (*mesh->texCoords())[texCoordIndex];
            // Invert the V components to convert the texture coordinate origin
            // from bottom-left (Choreonoid) to top-left (glTF)
            texCoords.insert(texCoords.end(), { t.x(), 1.0f - t.y() });
        }
        if(hasColors){
            const auto& c = (*mesh->colors())[colorIndex];
            colors.insert(colors.end(), { c.x(), c.y(), c.z() });
        }
    };

    if(!hasSeparateIndices){
        // The vertex attributes share the unified indices and can be output as they are
        const int n = orgVertices.size();
        vertices.reserve(n * 3);
        for(int i = 0; i < n; ++i){
            pushVertex(i, i, i, i);
        }
        triangleVertices = orgTriangleVertices;
    } else {
        /*
           The attributes are indexed separately in the mesh. Since glTF requires the
           unified vertex indexing, the vertices are expanded so that each combination
           of the attribute indices becomes a single vertex.
        */
        map<tuple<int, int, int, int>, int> vertexMap;
        triangleVertices.reserve(orgTriangleVertices.size());
        const int numCorners = orgTriangleVertices.size();
        for(int i = 0; i < numCorners; ++i){
            const int vi = orgTriangleVertices[i];
            const int ni = (hasNormals && !normalIndices.empty()) ? normalIndices[i] : vi;
            const int ti = (hasTexCoords && !texCoordIndices.empty()) ? texCoordIndices[i] : vi;
            const int ci = (hasColors && !colorIndices.empty()) ? colorIndices[i] : vi;
            const auto key = make_tuple(vi, ni, ti, ci);
            auto inserted = vertexMap.emplace(key, vertexMap.size());
            if(inserted.second){
                pushVertex(vi, ni, ti, ci);
            }
            triangleVertices.push_back(inserted.first->second);
        }
    }

    const int numVertices = vertices.size() / 3;
    primitive.position = addFloatAccessor(vertices, 3, "VEC3", true);
    if(hasNormals){
        primitive.normal = addFloatAccessor(normals, 3, "VEC3", false);
    }
    if(hasTexCoords){
        primitive.texCoord = addFloatAccessor(texCoords, 2, "VEC2", false);
    }
    if(hasColors){
        primitive.color = addFloatAccessor(colors, 3, "VEC3", false);
    }
    primitive.indices = addIndexAccessor(triangleVertices, numVertices);
}


int GLTFSceneWriter::Impl::getOrCreateMaterialIndex
(SgMaterial* material, SgTexture* shapeTexture, bool doubleSided)
{
    if(!material){
        if(!defaultMaterial){
            defaultMaterial = new SgMaterial;
        }
        material = defaultMaterial;
    }

    const auto key = make_tuple(material, shapeTexture, doubleSided);
    auto found = materialMap.find(key);
    if(found != materialMap.end()){
        return found->second;
    }

    MaterialDef def;
    def.material = material;
    def.doubleSided = doubleSided;
    if(auto pbr = material->pbrExtension()){
        if(pbr->emissiveStrength() != 1.0f){
            useEmissiveStrength = true;
        }
        if(pbr->baseColorTexture()){
            def.baseColorTexture = getOrCreateTextureIndex(pbr->baseColorTexture());
        }
        if(pbr->metallicRoughnessTexture()){
            def.metallicRoughnessTexture = getOrCreateTextureIndex(pbr->metallicRoughnessTexture());
        }
        if(pbr->normalTexture()){
            def.normalTexture = getOrCreateTextureIndex(pbr->normalTexture());
        }
        if(pbr->occlusionTexture()){
            def.occlusionTexture = getOrCreateTextureIndex(pbr->occlusionTexture());
        }
        if(pbr->emissiveTexture()){
            def.emissiveTexture = getOrCreateTextureIndex(pbr->emissiveTexture());
        }
    } else {
        def.isInferredMetal = isMetalLikeClassicMaterial(material);
        if(!def.isInferredMetal){
            // The specular color of the classic material is output using the
            // KHR_materials_specular extension. See emitMaterial for the details.
            useSpecularExtension = true;
        }
        if(shapeTexture){
            def.baseColorTexture = getOrCreateTextureIndex(shapeTexture);
        }
    }

    materials.push_back(std::move(def));
    const int index = materials.size() - 1;
    materialMap[key] = index;
    return index;
}


int GLTFSceneWriter::Impl::getOrCreateTextureIndex(SgTexture* texture)
{
    auto found = textureMap.find(texture);
    if(found != textureMap.end()){
        return found->second;
    }
    if(!texture->image() || texture->image()->empty()){
        textureMap[texture] = -1;
        return -1;
    }
    const int imageIndex = getOrCreateImageIndex(texture->image());
    if(imageIndex < 0){
        textureMap[texture] = -1;
        return -1;
    }
    TextureDef def;
    def.source = imageIndex;
    def.sampler = getOrCreateSamplerIndex(texture->repeatS(), texture->repeatT());
    textures.push_back(def);
    const int index = textures.size() - 1;
    textureMap[texture] = index;
    return index;
}


int GLTFSceneWriter::Impl::getOrCreateImageIndex(SgImage* image)
{
    auto found = imageMap.find(image);
    if(found != imageMap.end()){
        return found->second;
    }

    int index = -1;

    /*
       In the external image mode, the image is output as an image file into the
       output directory and referenced by the relative URI. The file is written
       in the ordinary top-down orientation, which matches the texture coordinates
       output by this writer. When the image cannot be made available as a file
       with a relative path (outputImageFile may return the absolute path of an
       original image file out of the output directory), the image is embedded
       in the binary data by the following codes to keep the output portable.
    */
    if(useExternalImages){
        string imageFile;
        if(self->outputImageFile(image, outputImageDirectory, imageFile)){
            filesystem::path imagePath(fromUTF8(imageFile));
            if(imagePath.is_relative()){
                ImageDef def;
                def.uri = encodeUri(toUTF8(imagePath.generic_string()));
                def.name = image->name();
                images.push_back(std::move(def));
                index = images.size() - 1;
            }
        }
    }

    /*
       When the original image file data (PNG or JPEG) is available, it is embedded
       as it is to avoid the deterioration in the file size and the encoding time
       caused by re-encoding. The original data is obtained from the source data
       attached to SgImage, or the original image file pointed by the uri of SgImage.
       Note that the original data keeps the original top-down orientation, which
       matches the texture coordinates output by this writer.
    */
    if(index < 0){
        if(auto sourceData = image->sourceData()){
            if(sourceData->mimeType() == "image/png" || sourceData->mimeType() == "image/jpeg"){
                ImageDef def;
                def.bufferView = addBufferView(sourceData->data().data(), sourceData->data().size(), 0);
                def.name = image->name();
                def.mimeType = sourceData->mimeType();
                images.push_back(std::move(def));
                index = images.size() - 1;
            }
        }
    }
    if(index < 0 && image->hasAbsoluteUri()){
        string path = image->absoluteUri();
        if(path.compare(0, 7, "file://") == 0){
            path = path.substr(7);
        }
        ifstream file(filesystem::path(fromUTF8(path)), ios::binary | ios::ate);
        if(file){
            const auto size = file.tellg();
            vector<uint8_t> data(size);
            file.seekg(0);
            file.read(reinterpret_cast<char*>(data.data()), size);
            if(file){
                const string mimeType = detectImageMimeType(data.data(), data.size());
                if(mimeType == "image/png" || mimeType == "image/jpeg"){
                    ImageDef def;
                    def.bufferView = addBufferView(data.data(), data.size(), 0);
                    def.name = image->name();
                    def.mimeType = mimeType;
                    images.push_back(std::move(def));
                    index = images.size() - 1;
                }
            }
        }
    }

    // Fallback: encode the image pixels into PNG
    if(index < 0){
        const Image& im = image->constImage();
        if(im.pixelType() != Image::UInt8){
            os() << formatR(_("Warning: Texture image \"{0}\" is not embedded because "
                              "its pixel format is not supported by glTF."),
                            image->name()) << endl;
        } else {
            vector<uint8_t> png;
            auto callback = [](void* context, void* data, int size){
                auto buffer = static_cast<vector<uint8_t>*>(context);
                auto p = static_cast<const uint8_t*>(data);
                buffer->insert(buffer->end(), p, p + size);
            };
            if(!stbi_write_png_to_func(
                   callback, &png, im.width(), im.height(), im.numComponents(),
                   im.pixels(), im.width() * im.numComponents())){
                os() << formatR(_("Warning: Texture image \"{0}\" cannot be encoded into PNG."),
                                image->name()) << endl;
            } else {
                ImageDef def;
                def.bufferView = addBufferView(png.data(), png.size(), 0);
                def.name = image->name();
                def.mimeType = "image/png";
                images.push_back(std::move(def));
                index = images.size() - 1;
            }
        }
    }

    imageMap[image] = index;
    return index;
}


int GLTFSceneWriter::Impl::getOrCreateSamplerIndex(bool repeatS, bool repeatT)
{
    const auto key = make_pair(repeatS, repeatT);
    auto found = samplerMap.find(key);
    if(found != samplerMap.end()){
        return found->second;
    }
    samplers.push_back({ repeatS, repeatT });
    const int index = samplers.size() - 1;
    samplerMap[key] = index;
    return index;
}


int GLTFSceneWriter::Impl::addBufferView(const void* data, size_t size, int target)
{
    while(binary.size() % 4 != 0){
        binary.push_back(0);
    }
    BufferViewDef def;
    def.offset = binary.size();
    def.length = size;
    def.target = target;
    const auto p = static_cast<const uint8_t*>(data);
    binary.insert(binary.end(), p, p + size);
    bufferViews.push_back(def);
    return bufferViews.size() - 1;
}


int GLTFSceneWriter::Impl::addFloatAccessor
(const vector<float>& values, int numComponentsPerElement, const char* type, bool withMinMax)
{
    AccessorDef def;
    def.bufferView = addBufferView(
        values.data(), values.size() * sizeof(float), GLTF_ARRAY_BUFFER);
    def.componentType = GLTF_FLOAT;
    def.count = values.size() / numComponentsPerElement;
    def.type = type;
    def.hasMinMax = withMinMax;
    if(withMinMax && numComponentsPerElement == 3){
        def.minValues.setConstant(numeric_limits<float>::max());
        def.maxValues.setConstant(-numeric_limits<float>::max());
        for(size_t i = 0; i < values.size(); i += 3){
            for(int j = 0; j < 3; ++j){
                def.minValues[j] = std::min(def.minValues[j], values[i + j]);
                def.maxValues[j] = std::max(def.maxValues[j], values[i + j]);
            }
        }
    }
    accessors.push_back(def);
    return accessors.size() - 1;
}


int GLTFSceneWriter::Impl::addIndexAccessor(const SgIndexArray& indices, int numVertices)
{
    AccessorDef def;
    if(numVertices <= 65535){
        vector<uint16_t> shortIndices(indices.begin(), indices.end());
        def.bufferView = addBufferView(
            shortIndices.data(), shortIndices.size() * sizeof(uint16_t), GLTF_ELEMENT_ARRAY_BUFFER);
        def.componentType = GLTF_UNSIGNED_SHORT;
    } else {
        vector<uint32_t> intIndices(indices.begin(), indices.end());
        def.bufferView = addBufferView(
            intIndices.data(), intIndices.size() * sizeof(uint32_t), GLTF_ELEMENT_ARRAY_BUFFER);
        def.componentType = GLTF_UNSIGNED_INT;
    }
    def.count = indices.size();
    def.type = "SCALAR";
    def.hasMinMax = false;
    accessors.push_back(def);
    return accessors.size() - 1;
}


void GLTFSceneWriter::Impl::emitTextureRef
(JsonWriter& json, const char* key, int index, const char* scaleKey, double scale)
{
    if(index < 0){
        return;
    }
    json.key(key);
    json.startObject();
    json.keyValue("index", index);
    if(scaleKey && scale != 1.0){
        json.keyValue(scaleKey, scale);
    }
    json.endObject();
}


void GLTFSceneWriter::Impl::emitMaterial(JsonWriter& json, const MaterialDef& def)
{
    auto material = def.material.get();

    json.startObject();
    if(!material->name().empty()){
        json.keyValue("name", material->name());
    }

    Vector3f baseColor;
    float alpha = 1.0f;
    float metallic;
    float roughness;
    Vector3f emissive;
    string alphaMode = "OPAQUE";
    double alphaCutoff = 0.5;
    double normalScale = 1.0;
    double occlusionStrength = 1.0;
    float emissiveStrength = 1.0f;

    if(auto pbr = material->pbrExtension()){
        baseColor = pbr->baseColorFactor();
        metallic = pbr->metallicFactor();
        roughness = pbr->roughnessFactor();
        emissive = pbr->emissiveFactor();
        emissiveStrength = pbr->emissiveStrength();
        normalScale = pbr->normalScale();
        occlusionStrength = pbr->occlusionStrength();
        switch(pbr->alphaMode()){
        case SgPbrMaterialExtension::Mask:
            alphaMode = "MASK";
            alphaCutoff = pbr->alphaCutoff();
            break;
        case SgPbrMaterialExtension::Blend:
            alphaMode = "BLEND";
            alpha = 1.0f - material->transparency();
            break;
        default:
            break;
        }
    } else {
        // Approximate conversion from the classic material model.
        // The metal detection is done by isMetalLikeClassicMaterial.
        roughness = std::clamp(1.0f - material->specularExponent() / 128.0f, 0.05f, 1.0f);
        emissive = material->emissiveColor().cwiseMin(1.0f).cwiseMax(0.0f);
        if(material->transparency() > 0.0f){
            alphaMode = "BLEND";
            alpha = 1.0f - material->transparency();
        }
        if(def.isInferredMetal){
            metallic = 1.0f;
            /*
               In the metallic-roughness model, the reflection color of a metal is
               carried by the base color. The specular color, which expresses the
               reflection color in the classic model, is therefore used as the base
               color. It is merged with the diffuse color by taking the maximum of
               each component so that the brightness expressed with either
               parameter is not lost.
            */
            baseColor = material->diffuseColor().cwiseMax(material->specularColor());
        } else {
            metallic = 0.0f;
            baseColor = material->diffuseColor();
        }
    }

    json.key("pbrMetallicRoughness");
    json.startObject();
    if(baseColor != Vector3f::Ones() || alpha != 1.0f){
        json.key("baseColorFactor");
        json.startArray();
        json.put(static_cast<double>(baseColor.x()));
        json.put(static_cast<double>(baseColor.y()));
        json.put(static_cast<double>(baseColor.z()));
        json.put(static_cast<double>(alpha));
        json.endArray();
    }
    if(metallic != 1.0f){
        json.keyValue("metallicFactor", static_cast<double>(metallic));
    }
    if(roughness != 1.0f){
        json.keyValue("roughnessFactor", static_cast<double>(roughness));
    }
    emitTextureRef(json, "baseColorTexture", def.baseColorTexture);
    emitTextureRef(json, "metallicRoughnessTexture", def.metallicRoughnessTexture);
    json.endObject();

    emitTextureRef(json, "normalTexture", def.normalTexture, "scale", normalScale);
    emitTextureRef(json, "occlusionTexture", def.occlusionTexture, "strength", occlusionStrength);
    emitTextureRef(json, "emissiveTexture", def.emissiveTexture);

    if(!emissive.isZero()){
        json.key("emissiveFactor");
        json.startArray();
        json.put(static_cast<double>(emissive.x()));
        json.put(static_cast<double>(emissive.y()));
        json.put(static_cast<double>(emissive.z()));
        json.endArray();
    }
    const bool writesClassicSpecular = !material->hasPbrExtension() && !def.isInferredMetal;
    if(emissiveStrength != 1.0f || writesClassicSpecular){
        json.key("extensions");
        json.startObject();
        if(emissiveStrength != 1.0f){
            json.key("KHR_materials_emissive_strength");
            json.startObject();
            json.keyValue("emissiveStrength", static_cast<double>(emissiveStrength));
            json.endObject();
        }
        if(writesClassicSpecular){
            /*
               The specular color of the classic material has no counterpart in the
               core metallic-roughness model and is stored using the
               KHR_materials_specular extension, which specifies the strength and
               the color of the specular reflection of a non-metal. The specular
               color factor is the specular color divided by 0.4, which is the
               inverse of the expression used by GLTFSceneLoader so that the
               original specular color is exactly restored when the file is loaded
               by Choreonoid. For the PBR renderers, the factor gives the specular
               reflectance (F0) of one tenth of the classic specular color, which
               lands on physically plausible values because classic specular colors
               are conventionally set around ten times the physical reflectance to
               compensate for the missing normalization of the Phong specular term.
               Note that the extension is only valid for non-metals; the reflection
               of a metal is fully expressed by the base color in the core
               specification.
            */
            const Vector3f factor = material->specularColor() / 0.4f;
            json.key("KHR_materials_specular");
            json.startObject();
            json.key("specularColorFactor");
            json.startArray();
            json.put(static_cast<double>(factor.x()));
            json.put(static_cast<double>(factor.y()));
            json.put(static_cast<double>(factor.z()));
            json.endArray();
            json.endObject();
        }
        json.endObject();
    }
    if(alphaMode != "OPAQUE"){
        json.keyValue("alphaMode", alphaMode);
        if(alphaMode == "MASK" && alphaCutoff != 0.5){
            json.keyValue("alphaCutoff", alphaCutoff);
        }
    }
    if(def.doubleSided){
        json.keyValue("doubleSided", true);
    }
    json.endObject();
}


string GLTFSceneWriter::Impl::emitJson(const string& binUri, size_t binSize)
{
    JsonWriter json;
    json.startObject();

    json.key("asset");
    json.startObject();
    json.keyValue("version", "2.0");
    json.keyValue("generator", "Choreonoid GLTFSceneWriter");
    json.endObject();

    if(useEmissiveStrength || useSpecularExtension){
        json.key("extensionsUsed");
        json.startArray();
        if(useEmissiveStrength){
            json.put("KHR_materials_emissive_strength");
        }
        if(useSpecularExtension){
            json.put("KHR_materials_specular");
        }
        json.endArray();
    }

    json.keyValue("scene", 0);
    json.key("scenes");
    json.startArray();
    json.startObject();
    json.key("nodes");
    json.startArray();
    json.put(static_cast<int>(nodes.size() - 1)); // the scene root node, which is registered last
    json.endArray();
    json.endObject();
    json.endArray();

    json.key("nodes");
    json.startArray();
    for(auto& node : nodes){
        json.startObject();
        if(!node.name.empty()){
            json.keyValue("name", node.name);
        }
        if(node.mesh >= 0){
            json.keyValue("mesh", node.mesh);
        }
        if(node.hasMatrix){
            json.key("matrix");
            json.startArray();
            for(int col = 0; col < 4; ++col){
                for(int row = 0; row < 4; ++row){
                    json.put(node.M.matrix()(row, col));
                }
            }
            json.endArray();
        }
        if(node.hasTranslation){
            json.key("translation");
            json.startArray();
            json.put(node.translation.x());
            json.put(node.translation.y());
            json.put(node.translation.z());
            json.endArray();
        }
        if(node.hasRotation){
            json.key("rotation");
            json.startArray();
            json.put(node.rotation.x());
            json.put(node.rotation.y());
            json.put(node.rotation.z());
            json.put(node.rotation.w());
            json.endArray();
        }
        if(node.hasScale){
            json.key("scale");
            json.startArray();
            json.put(node.scale.x());
            json.put(node.scale.y());
            json.put(node.scale.z());
            json.endArray();
        }
        if(!node.children.empty()){
            json.key("children");
            json.startArray();
            for(int child : node.children){
                json.put(child);
            }
            json.endArray();
        }
        json.endObject();
    }
    json.endArray();

    if(!meshes.empty()){
        json.key("meshes");
        json.startArray();
        for(auto& mesh : meshes){
            json.startObject();
            if(!mesh.name.empty()){
                json.keyValue("name", mesh.name);
            }
            json.key("primitives");
            json.startArray();
            for(auto& primitive : mesh.primitives){
                json.startObject();
                json.key("attributes");
                json.startObject();
                json.keyValue("POSITION", primitive.position);
                if(primitive.normal >= 0){
                    json.keyValue("NORMAL", primitive.normal);
                }
                if(primitive.texCoord >= 0){
                    json.keyValue("TEXCOORD_0", primitive.texCoord);
                }
                if(primitive.color >= 0){
                    json.keyValue("COLOR_0", primitive.color);
                }
                json.endObject();
                if(primitive.indices >= 0){
                    json.keyValue("indices", primitive.indices);
                }
                if(primitive.material >= 0){
                    json.keyValue("material", primitive.material);
                }
                if(primitive.mode != GLTF_TRIANGLES){
                    json.keyValue("mode", primitive.mode);
                }
                json.endObject();
            }
            json.endArray();
            json.endObject();
        }
        json.endArray();
    }

    if(!materials.empty()){
        json.key("materials");
        json.startArray();
        for(auto& material : materials){
            emitMaterial(json, material);
        }
        json.endArray();
    }

    if(!textures.empty()){
        json.key("textures");
        json.startArray();
        for(auto& texture : textures){
            json.startObject();
            json.keyValue("source", texture.source);
            json.keyValue("sampler", texture.sampler);
            json.endObject();
        }
        json.endArray();

        json.key("samplers");
        json.startArray();
        for(auto& sampler : samplers){
            json.startObject();
            json.keyValue("wrapS", sampler.repeatS ? GLTF_REPEAT : GLTF_CLAMP_TO_EDGE);
            json.keyValue("wrapT", sampler.repeatT ? GLTF_REPEAT : GLTF_CLAMP_TO_EDGE);
            json.endObject();
        }
        json.endArray();
    }

    if(!images.empty()){
        json.key("images");
        json.startArray();
        for(auto& image : images){
            json.startObject();
            if(!image.name.empty()){
                json.keyValue("name", image.name);
            }
            if(!image.uri.empty()){
                json.keyValue("uri", image.uri);
            } else {
                json.keyValue("mimeType", image.mimeType);
                json.keyValue("bufferView", image.bufferView);
            }
            json.endObject();
        }
        json.endArray();
    }

    if(!accessors.empty()){
        json.key("accessors");
        json.startArray();
        for(auto& accessor : accessors){
            json.startObject();
            json.keyValue("bufferView", accessor.bufferView);
            json.keyValue("componentType", accessor.componentType);
            json.keyValue("count", accessor.count);
            json.keyValue("type", accessor.type);
            if(accessor.hasMinMax){
                json.key("min");
                json.startArray();
                for(int i = 0; i < 3; ++i) json.put(static_cast<double>(accessor.minValues[i]));
                json.endArray();
                json.key("max");
                json.startArray();
                for(int i = 0; i < 3; ++i) json.put(static_cast<double>(accessor.maxValues[i]));
                json.endArray();
            }
            json.endObject();
        }
        json.endArray();
    }

    if(!bufferViews.empty()){
        json.key("bufferViews");
        json.startArray();
        for(auto& bufferView : bufferViews){
            json.startObject();
            json.keyValue("buffer", 0);
            json.keyValue("byteOffset", bufferView.offset);
            json.keyValue("byteLength", bufferView.length);
            if(bufferView.target != 0){
                json.keyValue("target", bufferView.target);
            }
            json.endObject();
        }
        json.endArray();

        json.key("buffers");
        json.startArray();
        json.startObject();
        if(!binUri.empty()){
            json.keyValue("uri", binUri);
        }
        json.keyValue("byteLength", binSize);
        json.endObject();
        json.endArray();
    }

    json.endObject();

    return std::move(json.text);
}
