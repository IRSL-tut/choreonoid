#include "ColladaSceneLoader.h"
#include "SceneDrawables.h"
#include "SceneLoader.h"
#include "SceneUtil.h"
#include "MeshUtil.h"
#include "EigenUtil.h"
#include "ImageIO.h"
#include "MeshFilter.h"
#include "NullOut.h"
#include "UTF8.h"
#include "Format.h"
#include <pugixml.hpp>
#include <fast_float/fast_float.h>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <charconv>
#include <cctype>
#include <cstring>
#include <optional>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace filesystem = std::filesystem;

namespace {

struct Registration {
    Registration(){
        SceneLoader::registerLoader(
            "dae",
            []() -> shared_ptr<AbstractSceneLoader> {
                return make_shared<ColladaSceneLoader>(); });
    }
} registration;

class LoadingException
{
public:
    LoadingException(const std::string& message) : message(message) { }
    std::string message;
};

void parseFloats(const char* text, vector<float>& out)
{
    out.clear();
    const char* p = text;
    const char* end = text + strlen(text);
    while(p < end){
        while(p < end && isspace(static_cast<unsigned char>(*p))){
            ++p;
        }
        if(p >= end){
            break;
        }
        float value;
        auto result = fast_float::from_chars(p, end, value);
        if(result.ec != std::errc()){
            throw LoadingException(_("An array of floating-point numbers cannot be parsed."));
        }
        out.push_back(value);
        p = result.ptr;
    }
}

void parseInts(const char* text, vector<int>& out)
{
    out.clear();
    const char* p = text;
    const char* end = text + strlen(text);
    while(p < end){
        while(p < end && isspace(static_cast<unsigned char>(*p))){
            ++p;
        }
        if(p >= end){
            break;
        }
        int value;
        auto result = std::from_chars(p, end, value);
        if(result.ec != std::errc()){
            throw LoadingException(_("An array of integers cannot be parsed."));
        }
        out.push_back(value);
        p = result.ptr;
    }
}

string trimmed(const char* text)
{
    string s(text);
    auto first = s.find_first_not_of(" \t\r\n");
    if(first == string::npos){
        return string();
    }
    auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
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

// Convert an image URI of the "file://" scheme to a plain file path
string removeFileScheme(const string& uri)
{
    if(uri.compare(0, 7, "file://") != 0){
        return uri;
    }
    string path = uri.substr(7);
    // "file:///C:/..." must be converted to "C:/..." on Windows
    if(path.size() >= 3 && path[0] == '/' && isalpha(static_cast<unsigned char>(path[1])) && path[2] == ':'){
        path = path.substr(1);
    }
    return path;
}

}

namespace cnoid {

class ColladaSceneLoader::Impl
{
public:
    ColladaSceneLoader* self;
    ostream* os_;
    ostream& os() { return *os_; }
    ImageIO imageIO;
    MeshFilter meshFilter;
    double defaultCreaseAngle;
    vector<filesystem::path> imageSearchDirectories;

    // Loading context
    filesystem::path directory; // directory of the file being loaded
    pugi::xml_document document;
    unordered_map<string, pugi::xml_node> idMap;
    unordered_map<string, SgVertexArrayPtr> vector3ArrayMap; // keyed by source id
    unordered_map<string, SgTexCoordArrayPtr> texCoordArrayMap; // keyed by source id
    unordered_map<string, SgNodePtr> libraryNodeMap; // keyed by node id
    unordered_map<string, SgNodePtr> geometryMap; // keyed by geometry id and material binding

    struct MaterialInfo {
        SgMaterialPtr material;
        SgTexturePtr texture; // diffuse texture
    };
    unordered_map<string, MaterialInfo> materialMap; // keyed by material id
    unordered_map<string, SgImagePtr> imageMap; // keyed by image id
    unordered_set<string> oneTimeWarnings;

    /*
       The transform accumulated from the node where a reflection (a transform whose
       determinant is negative) is found. While this is active, the transforms are
       baked into the mesh vertices instead of being kept in the scene graph, because
       the downstream subsystems assuming rotation-only transforms cannot correctly
       handle a reflection. See the comment in AssimpSceneLoader.cpp for details.
    */
    std::optional<Affine3f> bakeTransform;

    // Symbol to material id pairs given by a bind_material element
    typedef vector<pair<string, string>> MaterialBinding;

    struct PrimitiveInputs {
        int indexStride;
        int vertexOffset;
        int normalOffset;
        int texCoordOffset;
        int colorOffset;
        SgVertexArrayPtr vertices;
        SgNormalArrayPtr normals; // indexed by the NORMAL input of the primitive
        SgTexCoordArrayPtr texCoords; // indexed by the TEXCOORD input of the primitive
        SgColorArrayPtr colors; // indexed by the COLOR input of the primitive
        SgNormalArrayPtr vertexNormals; // given per vertex via the vertices element
        SgTexCoordArrayPtr vertexTexCoords;
        SgColorArrayPtr vertexColors;
    };

    Impl(ColladaSceneLoader* self);
    void clearLoadingContext();
    void warnOnce(const string& message);
    SgNode* load(const string& filename);
    void collectIds(pugi::xml_node node);
    pugi::xml_node resolveUrl(pugi::xml_node node, const char* attribute);
    SgNode* loadScene(pugi::xml_node collada);
    SgNode* convertNode(pugi::xml_node node, int depth);
    Affine3 readNodeTransform(pugi::xml_node node);
    void convertNodeContents(pugi::xml_node node, SgGroup* group, int depth);
    SgNode* convertInstanceGeometry(pugi::xml_node instance);
    SgNode* convertInstanceController(pugi::xml_node instance);
    MaterialBinding readMaterialBinding(pugi::xml_node instance);
    SgNode* getOrConvertGeometry(pugi::xml_node geometry, const MaterialBinding& binding);
    SgNode* convertGeometry(pugi::xml_node geometry, const MaterialBinding& binding);
    bool readPrimitiveInputs(pugi::xml_node primitive, PrimitiveInputs& inputs);
    SgVertexArray* getOrCreateVector3Array(pugi::xml_node source);
    SgTexCoordArray* getOrCreateTexCoordArray(pugi::xml_node source);
    bool readSourceValues(pugi::xml_node source, int minStride, vector<float>& values,
                          int& out_count, int& out_stride, int& out_offset);
    SgNode* convertMeshPrimitive(pugi::xml_node primitive, const MaterialBinding& binding);
    SgNode* convertLinePrimitive(pugi::xml_node primitive, const MaterialBinding& binding);
    void setMaterialToShape(SgShape* shape, pugi::xml_node primitive, const MaterialBinding& binding);
    const MaterialInfo* findMaterialInfo(pugi::xml_node primitive, const MaterialBinding& binding);
    const MaterialInfo& getOrCreateMaterial(const string& materialId);
    void readEffect(pugi::xml_node effect, MaterialInfo& info);
    SgTexture* readTexture(
        const unordered_map<string, pugi::xml_node>& paramMap, const string& samplerSid);
    SgImage* getOrCreateImage(const string& imageId);
};

}


ColladaSceneLoader::ColladaSceneLoader()
{
    impl = new Impl(this);
}


ColladaSceneLoader::Impl::Impl(ColladaSceneLoader* self)
    : self(self)
{
    os_ = &nullout();
    defaultCreaseAngle = 3.14159; // Generate fully smoothed normals by default

    /*
       The origin of the texture coordinates is bottom-left in COLLADA, which follows
       the OpenGL convention and is the same as that of Choreonoid (and OBJ and VRML).
       The image data are loaded upside down in the same way as the other loaders so
       that the data follow the image orientation convention used in Choreonoid.
    */
    imageIO.setUpsideDown(true);
}


ColladaSceneLoader::~ColladaSceneLoader()
{
    delete impl;
}


void ColladaSceneLoader::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
}


void ColladaSceneLoader::setDefaultCreaseAngle(double theta)
{
    impl->defaultCreaseAngle = theta;
}


void ColladaSceneLoader::addImageSearchDirectory(const std::string& directory)
{
    impl->imageSearchDirectories.emplace_back(fromUTF8(directory));
}


void ColladaSceneLoader::clearImageSearchDirectories()
{
    impl->imageSearchDirectories.clear();
}


SgNode* ColladaSceneLoader::load(const std::string& filename)
{
    SgNode* scene = insertTransformNodesToAdjustLengthUnitAndUpperAxis(impl->load(filename));
    if(scene){
        setFileUriInformationToScene(scene, filename);
    }
    return scene;
}


void ColladaSceneLoader::Impl::clearLoadingContext()
{
    document.reset();
    idMap.clear();
    vector3ArrayMap.clear();
    texCoordArrayMap.clear();
    libraryNodeMap.clear();
    geometryMap.clear();
    materialMap.clear();
    imageMap.clear();
    oneTimeWarnings.clear();
    bakeTransform.reset();
}


void ColladaSceneLoader::Impl::warnOnce(const string& message)
{
    if(oneTimeWarnings.insert(message).second){
        os() << message << endl;
    }
}


SgNode* ColladaSceneLoader::Impl::load(const string& filename)
{
    clearLoadingContext();

    SgNode* scene = nullptr;

    try {
        filesystem::path filepath(fromUTF8(filename));
        directory = filepath.parent_path();

        ifstream file(filepath, ios::binary);
        if(!file){
            throw LoadingException(formatR(_("The file cannot be opened. {0}"), strerror(errno)));
        }
        auto result = document.load(file);
        if(!result){
            throw LoadingException(
                formatR(_("The XML data cannot be parsed. {0}"), result.description()));
        }
        file.close();

        auto collada = document.document_element();
        if(strcmp(collada.name(), "COLLADA") != 0){
            throw LoadingException(_("The file does not have the COLLADA root element."));
        }

        collectIds(collada);

        scene = loadScene(collada);
    }
    catch(const LoadingException& ex){
        os() << formatR(_("COLLADA file \"{0}\" cannot be loaded: {1}"), filename, ex.message) << endl;
        scene = nullptr;
    }

    clearLoadingContext();

    return scene;
}


void ColladaSceneLoader::Impl::collectIds(pugi::xml_node node)
{
    for(auto child : node.children()){
        if(child.type() == pugi::node_element){
            if(auto idAttribute = child.attribute("id")){
                idMap.emplace(idAttribute.value(), child);
            }
            collectIds(child);
        }
    }
}


/**
   Resolves a URI fragment reference ("#id") stored in the specified attribute.
   An invalid node is returned when the reference cannot be resolved.
*/
pugi::xml_node ColladaSceneLoader::Impl::resolveUrl(pugi::xml_node node, const char* attribute)
{
    const char* url = node.attribute(attribute).value();
    if(url[0] != '#'){
        if(url[0] != '\0'){
            warnOnce(
                formatR(_("Warning: External document reference \"{0}\" is not supported."), url));
        }
        return pugi::xml_node();
    }
    auto it = idMap.find(url + 1);
    if(it == idMap.end()){
        warnOnce(formatR(_("Warning: The element referenced by \"{0}\" is not found."), url));
        return pugi::xml_node();
    }
    return it->second;
}


SgNode* ColladaSceneLoader::Impl::loadScene(pugi::xml_node collada)
{
    auto asset = collada.child("asset");
    double meter = asset.child("unit").attribute("meter").as_double(1.0);

    /*
       The up_axis element is intentionally ignored and the coordinates are always
       interpreted as Z-up, which is the convention of Choreonoid. This matches the
       behavior of AssimpSceneLoader (AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION)
       and the mesh handling of the ROS tools, which the existing robot models
       referencing Collada mesh files rely on.
    */
    string upAxis = trimmed(asset.child_value("up_axis"));
    if(!upAxis.empty() && upAxis != "Z_UP"){
        warnOnce(
            formatR(_("Warning: The up axis \"{0}\" is ignored and the coordinates are "
                      "interpreted as Z-up."), upAxis));
    }

    pugi::xml_node visualScene;
    if(auto instance = collada.child("scene").child("instance_visual_scene")){
        visualScene = resolveUrl(instance, "url");
    }
    if(!visualScene){
        // Fall back on the first visual scene when the scene element is missing
        visualScene = collada.child("library_visual_scenes").child("visual_scene");
    }
    if(!visualScene){
        throw LoadingException(_("The file does not have any visual scenes."));
    }

    SgGroupPtr sceneGroup = new SgGroup;
    const char* name = visualScene.attribute("name").value();
    if(name[0] == '\0'){
        name = visualScene.attribute("id").value();
    }
    sceneGroup->setName(name);

    for(auto node : visualScene.children("node")){
        if(auto sgNode = convertNode(node, 0)){
            sceneGroup->addChild(sgNode);
        }
    }

    SgNodePtr topNode;
    if(meter != 1.0){
        auto scale = new SgScaleTransform(meter);
        scale->addChild(sceneGroup);
        scale->setName(sceneGroup->name());
        topNode = scale;
    } else {
        topNode = sceneGroup;
    }
    sceneGroup.reset();

    return topNode.retn();
}


SgNode* ColladaSceneLoader::Impl::convertNode(pugi::xml_node node, int depth)
{
    if(depth > 512){
        throw LoadingException(_("The node hierarchy is too deep."));
    }

    const Affine3 T = readNodeTransform(node);

    /*
       When the node transform contains a reflection, the node and its descendants
       are converted into plain groups and the accumulated transform is baked into
       the mesh vertices in convertMeshPrimitive and convertLinePrimitive.
    */
    std::optional<Affine3f> prevBakeTransform = bakeTransform;
    SgGroupPtr group;
    SgGroup* contentGroup; // the node contents are added to this group owned by (or same as) group
    if(bakeTransform || T.linear().determinant() < 0.0){
        if(bakeTransform){
            bakeTransform = (*bakeTransform) * T.cast<float>();
        } else {
            bakeTransform = T.cast<float>();
        }
        group = new SgGroup;
        contentGroup = group;
    } else {
        SgGroupPtr top;
        SgGroupPtr inner;
        std::tie(top, inner) = createTransformNodeSet(T);
        group = top;
        contentGroup = inner;
    }

    const char* name = node.attribute("name").value();
    if(name[0] == '\0'){
        // Use the id when the human-readable name is not available
        name = node.attribute("id").value();
    }
    group->setName(name);

    convertNodeContents(node, contentGroup, depth);

    bakeTransform = prevBakeTransform;

    if(contentGroup->empty()){
        // Prune nodes that do not contain any shapes, such as camera and light nodes
        return nullptr;
    }

    return group.retn();
}


/**
   Reads the transformation elements of the specified node element. The elements
   are accumulated in the document order according to the COLLADA specification.
*/
Affine3 ColladaSceneLoader::Impl::readNodeTransform(pugi::xml_node node)
{
    Affine3 T = Affine3::Identity();
    vector<float> values;

    for(auto element : node.children()){
        const char* elementName = element.name();
        if(strcmp(elementName, "matrix") == 0){
            parseFloats(element.child_value(), values);
            if(values.size() != 16){
                throw LoadingException(_("A matrix element does not have 16 values."));
            }
            Affine3 M;
            for(int row = 0; row < 4; ++row){
                for(int col = 0; col < 4; ++col){
                    // The values of the COLLADA matrix are stored in the row-major order
                    M.matrix()(row, col) = values[row * 4 + col];
                }
            }
            T = T * M;
        } else if(strcmp(elementName, "translate") == 0){
            parseFloats(element.child_value(), values);
            if(values.size() == 3){
                T = T * Eigen::Translation3d(values[0], values[1], values[2]);
            }
        } else if(strcmp(elementName, "rotate") == 0){
            parseFloats(element.child_value(), values);
            if(values.size() == 4){
                Vector3 axis(values[0], values[1], values[2]);
                if(axis.norm() > 1.0e-12){
                    T = T * AngleAxis(radian(values[3]), axis.normalized());
                }
            }
        } else if(strcmp(elementName, "scale") == 0){
            parseFloats(element.child_value(), values);
            if(values.size() == 3){
                T = T * Eigen::Scaling(
                    static_cast<double>(values[0]),
                    static_cast<double>(values[1]),
                    static_cast<double>(values[2]));
            }
        } else if(strcmp(elementName, "lookat") == 0 || strcmp(elementName, "skew") == 0){
            warnOnce(
                formatR(_("Warning: The \"{0}\" transformation elements are not supported "
                          "and ignored."), elementName));
        }
    }

    return T;
}


void ColladaSceneLoader::Impl::convertNodeContents(pugi::xml_node node, SgGroup* group, int depth)
{
    for(auto element : node.children()){
        const char* elementName = element.name();
        if(strcmp(elementName, "node") == 0){
            if(auto sgNode = convertNode(element, depth + 1)){
                group->addChild(sgNode);
            }
        } else if(strcmp(elementName, "instance_node") == 0){
            if(auto libraryNode = resolveUrl(element, "url")){
                if(bakeTransform){
                    // The conversion result depends on the transform being baked
                    // into the vertices, so the library node cache is bypassed
                    if(auto sgNode = convertNode(libraryNode, depth + 1)){
                        group->addChild(sgNode);
                    }
                } else {
                    const string id = libraryNode.attribute("id").value();
                    auto found = libraryNodeMap.find(id);
                    if(found != libraryNodeMap.end()){
                        if(found->second){
                            group->addChild(found->second);
                        }
                    } else {
                        SgNodePtr sgNode = convertNode(libraryNode, depth + 1);
                        if(!id.empty()){
                            libraryNodeMap[id] = sgNode;
                        }
                        if(sgNode){
                            group->addChild(sgNode);
                        }
                    }
                }
            }
        } else if(strcmp(elementName, "instance_geometry") == 0){
            if(auto sgNode = convertInstanceGeometry(element)){
                group->addChild(sgNode);
            }
        } else if(strcmp(elementName, "instance_controller") == 0){
            if(auto sgNode = convertInstanceController(element)){
                group->addChild(sgNode);
            }
        }
        // The instance_camera and instance_light elements are silently skipped
    }
}


SgNode* ColladaSceneLoader::Impl::convertInstanceGeometry(pugi::xml_node instance)
{
    auto geometry = resolveUrl(instance, "url");
    if(!geometry){
        return nullptr;
    }
    return getOrConvertGeometry(geometry, readMaterialBinding(instance));
}


/**
   Converts a controller instance into the static shape of its base geometry.
   Skinning and morphing are not supported, but the base geometries are loaded
   so that models using controllers can at least be displayed.
*/
SgNode* ColladaSceneLoader::Impl::convertInstanceController(pugi::xml_node instance)
{
    auto element = resolveUrl(instance, "url");

    // Follow the controller chain (e.g. a morph inside a skin) to the base geometry
    int numFollowedLinks = 0;
    while(element && strcmp(element.name(), "controller") == 0){
        if(numFollowedLinks++ > 8){
            return nullptr;
        }
        pugi::xml_node source = element.child("skin");
        if(!source){
            source = element.child("morph");
        }
        if(!source){
            return nullptr;
        }
        element = resolveUrl(source, "source");
    }
    if(!element || strcmp(element.name(), "geometry") != 0){
        return nullptr;
    }

    warnOnce(_("Warning: Skinning and morphing are not supported and the base geometries are used."));

    return getOrConvertGeometry(element, readMaterialBinding(instance));
}


ColladaSceneLoader::Impl::MaterialBinding ColladaSceneLoader::Impl::readMaterialBinding
(pugi::xml_node instance)
{
    MaterialBinding binding;
    auto techniqueCommon = instance.child("bind_material").child("technique_common");
    for(auto instanceMaterial : techniqueCommon.children("instance_material")){
        const char* symbol = instanceMaterial.attribute("symbol").value();
        const char* target = instanceMaterial.attribute("target").value();
        if(symbol[0] != '\0' && target[0] == '#'){
            binding.emplace_back(symbol, target + 1);
        }
    }
    sort(binding.begin(), binding.end());
    return binding;
}


SgNode* ColladaSceneLoader::Impl::getOrConvertGeometry
(pugi::xml_node geometry, const MaterialBinding& binding)
{
    /*
       While a reflection is being baked into the mesh vertices, the conversion
       result depends on the transform accumulated in the instantiation context,
       so the cache must be bypassed.
    */
    if(bakeTransform){
        return convertGeometry(geometry, binding);
    }

    /*
       The converted geometries are cached and shared between the geometry instances.
       The cache key includes the material binding because the same geometry can be
       instantiated with different materials.
    */
    string key = geometry.attribute("id").value();
    if(key.empty()){
        return convertGeometry(geometry, binding);
    }
    for(auto& symbolAndMaterial : binding){
        key += "|";
        key += symbolAndMaterial.first;
        key += "=";
        key += symbolAndMaterial.second;
    }
    auto found = geometryMap.find(key);
    if(found != geometryMap.end()){
        return found->second;
    }
    SgNodePtr sgNode = convertGeometry(geometry, binding);
    geometryMap[key] = sgNode;
    return sgNode.retn();
}


SgNode* ColladaSceneLoader::Impl::convertGeometry
(pugi::xml_node geometry, const MaterialBinding& binding)
{
    auto mesh = geometry.child("mesh");
    if(!mesh){
        warnOnce(
            _("Warning: Geometries other than the mesh type (convex_mesh, spline, brep) "
              "are not supported and skipped."));
        return nullptr;
    }

    string name = geometry.attribute("name").value();
    if(name.empty()){
        name = geometry.attribute("id").value();
    }

    SgGroupPtr group = new SgGroup;
    group->setName(name);

    for(auto primitive : mesh.children()){
        const char* primitiveName = primitive.name();
        SgNodePtr sgNode;
        if(strcmp(primitiveName, "triangles") == 0 ||
           strcmp(primitiveName, "polylist") == 0 ||
           strcmp(primitiveName, "polygons") == 0){
            sgNode = convertMeshPrimitive(primitive, binding);
        } else if(strcmp(primitiveName, "lines") == 0 ||
                  strcmp(primitiveName, "linestrips") == 0){
            sgNode = convertLinePrimitive(primitive, binding);
        } else if(strcmp(primitiveName, "tristrips") == 0 ||
                  strcmp(primitiveName, "trifans") == 0){
            warnOnce(
                formatR(_("Warning: The \"{0}\" primitives are not supported and skipped."),
                        primitiveName));
        }
        if(sgNode){
            group->addChild(sgNode);
        }
    }

    if(group->empty()){
        return nullptr;
    }
    if(group->numChildren() == 1){
        SgNodePtr child = group->child(0);
        if(child->name().empty()){
            child->setName(name);
        }
        group->clearChildren();
        return child.retn();
    }

    // Give distinctive names to the shapes of the multi-primitive geometry
    for(int i = 0; i < group->numChildren(); ++i){
        auto child = group->child(i);
        if(child->name().empty() && !name.empty()){
            child->setName(formatC("{0}_{1}", name, i));
        }
    }

    return group.retn();
}


bool ColladaSceneLoader::Impl::readPrimitiveInputs(pugi::xml_node primitive, PrimitiveInputs& inputs)
{
    inputs.indexStride = 1;
    inputs.vertexOffset = -1;
    inputs.normalOffset = -1;
    inputs.texCoordOffset = -1;
    inputs.colorOffset = -1;

    for(auto input : primitive.children("input")){
        const char* semantic = input.attribute("semantic").value();
        const int offset = input.attribute("offset").as_int(0);
        inputs.indexStride = std::max(inputs.indexStride, offset + 1);

        if(strcmp(semantic, "VERTEX") == 0){
            inputs.vertexOffset = offset;
            auto vertices = resolveUrl(input, "source");
            for(auto vertexInput : vertices.children("input")){
                const char* vertexSemantic = vertexInput.attribute("semantic").value();
                auto source = resolveUrl(vertexInput, "source");
                if(!source){
                    continue;
                }
                if(strcmp(vertexSemantic, "POSITION") == 0){
                    inputs.vertices = getOrCreateVector3Array(source);
                } else if(strcmp(vertexSemantic, "NORMAL") == 0){
                    inputs.vertexNormals = getOrCreateVector3Array(source);
                } else if(strcmp(vertexSemantic, "TEXCOORD") == 0){
                    inputs.vertexTexCoords = getOrCreateTexCoordArray(source);
                } else if(strcmp(vertexSemantic, "COLOR") == 0){
                    inputs.vertexColors = getOrCreateVector3Array(source);
                }
            }
        } else if(strcmp(semantic, "NORMAL") == 0){
            if(inputs.normalOffset < 0){
                if(auto source = resolveUrl(input, "source")){
                    inputs.normals = getOrCreateVector3Array(source);
                    inputs.normalOffset = offset;
                }
            }
        } else if(strcmp(semantic, "TEXCOORD") == 0){
            if(inputs.texCoordOffset < 0){
                if(auto source = resolveUrl(input, "source")){
                    inputs.texCoords = getOrCreateTexCoordArray(source);
                    inputs.texCoordOffset = offset;
                }
            } else {
                warnOnce(_("Warning: Only the first texture coordinate set is supported."));
            }
        } else if(strcmp(semantic, "COLOR") == 0){
            if(inputs.colorOffset < 0){
                if(auto source = resolveUrl(input, "source")){
                    inputs.colors = getOrCreateVector3Array(source);
                    inputs.colorOffset = offset;
                }
            }
        }
    }

    if(!inputs.vertices){
        warnOnce(_("Warning: A primitive without the vertex positions is skipped."));
        return false;
    }
    return true;
}


bool ColladaSceneLoader::Impl::readSourceValues
(pugi::xml_node source, int minStride, vector<float>& values,
 int& out_count, int& out_stride, int& out_offset)
{
    auto accessor = source.child("technique_common").child("accessor");
    if(!accessor){
        return false;
    }
    auto floatArray = resolveUrl(accessor, "source");
    if(!floatArray || strcmp(floatArray.name(), "float_array") != 0){
        return false;
    }

    parseFloats(floatArray.child_value(), values);

    out_count = accessor.attribute("count").as_int(0);
    out_stride = accessor.attribute("stride").as_int(1);
    out_offset = accessor.attribute("offset").as_int(0);

    if(out_count < 0 || out_stride < minStride || out_offset < 0){
        return false;
    }
    // Check if the accessed range is within the value array
    if(out_count > 0 &&
       static_cast<size_t>(out_offset) + static_cast<size_t>(out_count - 1) * out_stride + minStride
       > values.size()){
        return false;
    }
    return true;
}


SgVertexArray* ColladaSceneLoader::Impl::getOrCreateVector3Array(pugi::xml_node source)
{
    const string id = source.attribute("id").value();
    auto found = vector3ArrayMap.find(id);
    if(found != vector3ArrayMap.end()){
        return found->second;
    }

    SgVertexArrayPtr array;
    vector<float> values;
    int count, stride, offset;
    if(readSourceValues(source, 3, values, count, stride, offset)){
        array = new SgVertexArray(count);
        for(int i = 0; i < count; ++i){
            const size_t base = offset + static_cast<size_t>(i) * stride;
            (*array)[i] << values[base], values[base + 1], values[base + 2];
        }
    } else {
        warnOnce(formatR(_("Warning: The source \"{0}\" cannot be read as 3D vectors."), id));
    }

    vector3ArrayMap[id] = array;
    return array;
}


SgTexCoordArray* ColladaSceneLoader::Impl::getOrCreateTexCoordArray(pugi::xml_node source)
{
    const string id = source.attribute("id").value();
    auto found = texCoordArrayMap.find(id);
    if(found != texCoordArrayMap.end()){
        return found->second;
    }

    SgTexCoordArrayPtr array;
    vector<float> values;
    int count, stride, offset;
    if(readSourceValues(source, 2, values, count, stride, offset)){
        array = new SgTexCoordArray(count);
        for(int i = 0; i < count; ++i){
            const size_t base = offset + static_cast<size_t>(i) * stride;
            (*array)[i] << values[base], values[base + 1];
        }
    } else {
        warnOnce(formatR(_("Warning: The source \"{0}\" cannot be read as 2D vectors."), id));
    }

    texCoordArrayMap[id] = array;
    return array;
}


SgNode* ColladaSceneLoader::Impl::convertMeshPrimitive
(pugi::xml_node primitive, const MaterialBinding& binding)
{
    PrimitiveInputs inputs;
    if(!readPrimitiveInputs(primitive, inputs)){
        return nullptr;
    }
    const int indexStride = inputs.indexStride;

    // Gather the vertex indices and the number of the vertices of each face
    vector<int> corners;
    vector<int> faceSizes;
    const char* primitiveName = primitive.name();
    if(strcmp(primitiveName, "triangles") == 0){
        parseInts(primitive.child_value("p"), corners);
        const int numFaces = corners.size() / (3 * indexStride);
        faceSizes.assign(numFaces, 3);
    } else if(strcmp(primitiveName, "polylist") == 0){
        parseInts(primitive.child_value("vcount"), faceSizes);
        parseInts(primitive.child_value("p"), corners);
    } else { // polygons
        vector<int> polygon;
        for(auto p : primitive.children("p")){
            parseInts(p.child_value(), polygon);
            corners.insert(corners.end(), polygon.begin(), polygon.end());
            faceSizes.push_back(polygon.size() / indexStride);
        }
        if(primitive.child("ph")){
            warnOnce(_("Warning: Polygons with holes are not supported and the holes are ignored."));
            for(auto ph : primitive.children("ph")){
                parseInts(ph.child_value("p"), polygon);
                corners.insert(corners.end(), polygon.begin(), polygon.end());
                faceSizes.push_back(polygon.size() / indexStride);
            }
        }
    }

    size_t numCorners = 0;
    for(int faceSize : faceSizes){
        numCorners += faceSize;
    }
    if(numCorners == 0){
        return nullptr;
    }
    if(numCorners * indexStride != corners.size()){
        throw LoadingException(
            _("The number of the indices of a primitive does not match its vertex counts."));
    }

    SgShapePtr shape = new SgShape;
    auto mesh = shape->getOrCreateMesh();
    mesh->setVertices(inputs.vertices);

    SgNormalArray* normals = nullptr;
    if(inputs.normals){
        normals = mesh->setNormals(inputs.normals);
    } else if(inputs.vertexNormals){
        mesh->setNormals(inputs.vertexNormals);
    }
    SgTexCoordArray* texCoords = nullptr;
    if(inputs.texCoords){
        texCoords = mesh->setTexCoords(inputs.texCoords);
    } else if(inputs.vertexTexCoords){
        mesh->setTexCoords(inputs.vertexTexCoords);
    }
    SgColorArray* colors = nullptr;
    if(inputs.colors){
        colors = mesh->setColors(inputs.colors);
    } else if(inputs.vertexColors){
        mesh->setColors(inputs.vertexColors);
    }

    const int numVertices = inputs.vertices->size();
    auto& triangleVertices = mesh->triangleVertices();
    auto& normalIndices = mesh->normalIndices();
    auto& texCoordIndices = mesh->texCoordIndices();
    auto& colorIndices = mesh->colorIndices();

    size_t numTriangles = 0;
    for(int faceSize : faceSizes){
        if(faceSize >= 3){
            numTriangles += faceSize - 2;
        }
    }
    triangleVertices.reserve(numTriangles * 3);

    auto addCorner = [&](size_t cornerIndex){
        const size_t base = cornerIndex * indexStride;
        const int vertexIndex = corners[base + inputs.vertexOffset];
        if(vertexIndex < 0 || vertexIndex >= numVertices){
            throw LoadingException(_("A primitive has an out-of-range vertex index."));
        }
        triangleVertices.push_back(vertexIndex);
        if(normals){
            const int index = corners[base + inputs.normalOffset];
            if(index < 0 || index >= static_cast<int>(normals->size())){
                throw LoadingException(_("A primitive has an out-of-range normal index."));
            }
            normalIndices.push_back(index);
        }
        if(texCoords){
            const int index = corners[base + inputs.texCoordOffset];
            if(index < 0 || index >= static_cast<int>(texCoords->size())){
                throw LoadingException(_("A primitive has an out-of-range texture coordinate index."));
            }
            texCoordIndices.push_back(index);
        }
        if(colors){
            const int index = corners[base + inputs.colorOffset];
            if(index < 0 || index >= static_cast<int>(colors->size())){
                throw LoadingException(_("A primitive has an out-of-range color index."));
            }
            colorIndices.push_back(index);
        }
    };

    // Convert each face into triangles as a triangle fan
    size_t faceTop = 0;
    for(int faceSize : faceSizes){
        for(int i = 1; i < faceSize - 1; ++i){
            addCorner(faceTop);
            addCorner(faceTop + i);
            addCorner(faceTop + i + 1);
        }
        faceTop += faceSize;
    }

    if(bakeTransform){
        /*
           The vertex and normal arrays can be shared with the other non-baked
           instances via vector3ArrayMap, so they are duplicated before the
           accumulated transform is baked into them. When the mesh does not have
           normals, bakeTransformIntoMesh flips the winding according to the
           determinant sign and the normals consistent with the baked vertices
           are generated below.
        */
        mesh->setVertices(new SgVertexArray(*mesh->vertices()));
        if(mesh->hasNormals()){
            mesh->setNormals(new SgNormalArray(*mesh->normals()));
        }
        bakeTransformIntoMesh(mesh, *bakeTransform);
    }

    if(!mesh->normals()){
        meshFilter.generateNormals(mesh, defaultCreaseAngle);
    }
    mesh->updateBoundingBox();

    setMaterialToShape(shape, primitive, binding);

    return shape.retn();
}


SgNode* ColladaSceneLoader::Impl::convertLinePrimitive
(pugi::xml_node primitive, const MaterialBinding& binding)
{
    PrimitiveInputs inputs;
    if(!readPrimitiveInputs(primitive, inputs)){
        return nullptr;
    }
    const int indexStride = inputs.indexStride;
    const int numVertices = inputs.vertices->size();

    SgLineSetPtr lineSet = new SgLineSet;
    lineSet->setVertices(inputs.vertices);
    auto& lineVertexIndices = lineSet->lineVertexIndices();

    auto extractVertexIndices = [&](const vector<int>& corners, vector<int>& out_indices){
        const size_t n = corners.size() / indexStride;
        out_indices.clear();
        out_indices.reserve(n);
        for(size_t i = 0; i < n; ++i){
            const int index = corners[i * indexStride + inputs.vertexOffset];
            if(index < 0 || index >= numVertices){
                throw LoadingException(_("A primitive has an out-of-range vertex index."));
            }
            out_indices.push_back(index);
        }
    };

    vector<int> corners;
    vector<int> indices;
    if(strcmp(primitive.name(), "lines") == 0){
        parseInts(primitive.child_value("p"), corners);
        extractVertexIndices(corners, indices);
        lineVertexIndices.assign(indices.begin(), indices.begin() + indices.size() / 2 * 2);
    } else { // linestrips
        for(auto p : primitive.children("p")){
            parseInts(p.child_value(), corners);
            extractVertexIndices(corners, indices);
            for(size_t i = 0; i + 1 < indices.size(); ++i){
                lineVertexIndices.push_back(indices[i]);
                lineVertexIndices.push_back(indices[i + 1]);
            }
        }
    }

    if(lineVertexIndices.empty()){
        return nullptr;
    }

    if(bakeTransform){
        // Only the vertex positions need baking for line primitives.
        // The shared vertex array is duplicated in the same way as in the
        // mesh primitive conversion.
        auto bakedVertices = new SgVertexArray(*lineSet->vertices());
        transformVertices(*bakedVertices, *bakeTransform);
        lineSet->setVertices(bakedVertices);
    }

    lineSet->updateBoundingBox();

    if(auto materialInfo = findMaterialInfo(primitive, binding)){
        lineSet->setMaterial(materialInfo->material);
    }

    return lineSet.retn();
}


void ColladaSceneLoader::Impl::setMaterialToShape
(SgShape* shape, pugi::xml_node primitive, const MaterialBinding& binding)
{
    if(auto materialInfo = findMaterialInfo(primitive, binding)){
        shape->setMaterial(materialInfo->material);
        if(materialInfo->texture && shape->mesh()->texCoords()){
            shape->setTexture(materialInfo->texture);
        }
    }
}


const ColladaSceneLoader::Impl::MaterialInfo* ColladaSceneLoader::Impl::findMaterialInfo
(pugi::xml_node primitive, const MaterialBinding& binding)
{
    const char* symbol = primitive.attribute("material").value();
    if(symbol[0] == '\0'){
        return nullptr;
    }
    string materialId;
    for(auto& symbolAndMaterial : binding){
        if(symbolAndMaterial.first == symbol){
            materialId = symbolAndMaterial.second;
            break;
        }
    }
    if(materialId.empty()){
        // Some files use material ids directly as the material symbols
        if(idMap.find(symbol) != idMap.end()){
            materialId = symbol;
        } else {
            warnOnce(
                formatR(_("Warning: Material symbol \"{0}\" is not found in the material "
                          "bindings."), symbol));
            return nullptr;
        }
    }
    return &getOrCreateMaterial(materialId);
}


const ColladaSceneLoader::Impl::MaterialInfo& ColladaSceneLoader::Impl::getOrCreateMaterial
(const string& materialId)
{
    auto found = materialMap.find(materialId);
    if(found != materialMap.end()){
        return found->second;
    }

    MaterialInfo info;
    info.material = new SgMaterial;

    auto it = idMap.find(materialId);
    if(it != idMap.end() && strcmp(it->second.name(), "material") == 0){
        auto material = it->second;
        const char* name = material.attribute("name").value();
        if(name[0] == '\0'){
            name = material.attribute("id").value();
        }
        info.material->setName(name);

        if(auto effect = resolveUrl(material.child("instance_effect"), "url")){
            readEffect(effect, info);
        }
    } else {
        warnOnce(formatR(_("Warning: Material \"{0}\" is not found."), materialId));
    }

    return materialMap.emplace(materialId, std::move(info)).first->second;
}


void ColladaSceneLoader::Impl::readEffect(pugi::xml_node effect, MaterialInfo& info)
{
    auto profile = effect.child("profile_COMMON");
    if(!profile){
        warnOnce(
            _("Warning: Effects without the common profile are not supported and the "
              "default material is used."));
        return;
    }

    // Collect the newparam elements (texture samplers and surfaces) of the profile
    unordered_map<string, pugi::xml_node> paramMap;
    auto technique = profile.child("technique");
    for(auto scope : { profile, technique }){
        for(auto newparam : scope.children("newparam")){
            const char* sid = newparam.attribute("sid").value();
            if(sid[0] != '\0'){
                paramMap[sid] = newparam;
            }
        }
    }

    pugi::xml_node shader;
    for(const char* type : { "phong", "blinn", "lambert", "constant" }){
        if((shader = technique.child(type))){
            break;
        }
    }
    if(!shader){
        return;
    }
    const bool isConstant = (strcmp(shader.name(), "constant") == 0);

    auto material = info.material;
    vector<float> values;

    // Reads a color parameter. Returns true if a color value is given, and sets
    // out_texture if the parameter is given as a texture.
    auto readColor = [&](const char* key, Vector4f& out_color, pugi::xml_node* out_texture){
        auto parameter = shader.child(key);
        if(!parameter){
            return false;
        }
        if(out_texture){
            if(auto texture = parameter.child("texture")){
                *out_texture = texture;
                return false;
            }
        }
        auto color = parameter.child("color");
        if(!color){
            return false;
        }
        parseFloats(color.child_value(), values);
        if(values.size() < 3){
            return false;
        }
        out_color << values[0], values[1], values[2], (values.size() >= 4 ? values[3] : 1.0f);
        return true;
    };

    auto readFloat = [&](pugi::xml_node parameter, float& out_value){
        auto floatElement = parameter.child("float");
        if(!floatElement){
            return false;
        }
        parseFloats(floatElement.child_value(), values);
        if(values.empty()){
            return false;
        }
        out_value = values[0];
        return true;
    };

    Vector4f color;
    Vector3f diffuseColor = Vector3f::Zero();
    bool hasDiffuseColor = false;
    pugi::xml_node diffuseTexture;

    if(readColor("diffuse", color, &diffuseTexture)){
        diffuseColor = color.head<3>();
        hasDiffuseColor = true;
        material->setDiffuseColor(diffuseColor);
    }
    if(diffuseTexture){
        info.texture = readTexture(paramMap, diffuseTexture.attribute("texture").value());
        if(info.texture && !hasDiffuseColor){
            // Let the texture colors appear as they are
            material->setDiffuseColor(Vector3f(1.0f, 1.0f, 1.0f));
        }
    }
    if(readColor("ambient", color, nullptr)){
        // SgMaterial expresses the ambient color as the intensity relative to the diffuse color
        const float diffuseSum = diffuseColor.sum();
        float intensity = (diffuseSum > 0.0f) ? (color.head<3>().sum() / diffuseSum) : 0.0f;
        material->setAmbientIntensity(std::min(intensity, 1.0f));
    }
    bool hasSpecularColor = false;
    if(readColor("specular", color, nullptr)){
        material->setSpecularColor(Vector3f(color.head<3>()));
        hasSpecularColor = true;
    }
    if(readColor("emission", color, nullptr)){
        material->setEmissiveColor(Vector3f(color.head<3>()));
        if(isConstant){
            material->setDiffuseColor(Vector3f::Zero());
        }
    }
    float shininess;
    if(readFloat(shader.child("shininess"), shininess)){
        if(shininess > 0.0f){
            material->setSpecularExponent(shininess);
        } else if(hasSpecularColor){
            /*
               A shininess of zero makes the pow(x, shininess) factor of the specular
               term constantly one, which adds the full specular color uniformly to the
               whole surface and washes the model out. Exporters (e.g. OpenCOLLADA for
               3ds Max) often write zero here, which is interpreted as "no specular
               highlight" rather than as an actual exponent.
            */
            material->setSpecularColor(Vector3f::Zero());
        }
    }

    // Transparency
    Vector4f transparent(1.0f, 1.0f, 1.0f, 1.0f);
    float transparency = 1.0f;
    bool hasTransparent = readColor("transparent", transparent, nullptr);
    bool hasTransparency = readFloat(shader.child("transparency"), transparency);
    if(hasTransparent || hasTransparency){
        const char* opaque = shader.child("transparent").attribute("opaque").value();
        if(opaque[0] == '\0'){
            opaque = "A_ONE";
        }
        // Luminance defined by the COLLADA specification for the RGB opacity modes
        const float luminance =
            0.212671f * transparent[0] + 0.715160f * transparent[1] + 0.072169f * transparent[2];
        float opacity;
        if(strcmp(opaque, "RGB_ZERO") == 0){
            opacity = 1.0f - luminance * transparency;
        } else if(strcmp(opaque, "RGB_ONE") == 0){
            opacity = luminance * transparency;
        } else if(strcmp(opaque, "A_ZERO") == 0){
            opacity = 1.0f - transparent[3] * transparency;
        } else { // A_ONE (default)
            opacity = transparent[3] * transparency;
        }
        opacity = std::min(std::max(opacity, 0.0f), 1.0f);
        if(opacity <= 0.0f){
            // Some exporters write a fully transparent material for an opaque one.
            // Assume such a material is opaque as the other loaders do.
            opacity = 1.0f;
        }
        material->setTransparency(1.0f - opacity);
    }
}


SgTexture* ColladaSceneLoader::Impl::readTexture
(const unordered_map<string, pugi::xml_node>& paramMap, const string& samplerSid)
{
    if(samplerSid.empty()){
        return nullptr;
    }

    string imageId;
    bool repeatS = true;
    bool repeatT = true;

    auto found = paramMap.find(samplerSid);
    if(found != paramMap.end()){
        auto sampler = found->second.child("sampler2D");
        if(!sampler){
            return nullptr;
        }
        if(auto instanceImage = sampler.child("instance_image")){
            // The COLLADA 1.5 style direct image reference
            const char* url = instanceImage.attribute("url").value();
            if(url[0] == '#'){
                imageId = url + 1;
            }
        } else {
            // The COLLADA 1.4 style sampler and surface indirection
            string surfaceSid = trimmed(sampler.child_value("source"));
            auto foundSurface = paramMap.find(surfaceSid);
            if(foundSurface != paramMap.end()){
                imageId = trimmed(foundSurface->second.child("surface").child_value("init_from"));
            }
        }
        string wrapS = trimmed(sampler.child_value("wrap_s"));
        string wrapT = trimmed(sampler.child_value("wrap_t"));
        repeatS = wrapS.empty() || wrapS == "WRAP" || wrapS == "MIRROR";
        repeatT = wrapT.empty() || wrapT == "WRAP" || wrapT == "MIRROR";
    } else {
        // Some files refer to an image id directly from the texture element
        imageId = samplerSid;
    }

    if(imageId.empty()){
        return nullptr;
    }
    auto image = getOrCreateImage(imageId);
    if(!image){
        return nullptr;
    }

    auto texture = new SgTexture;
    texture->setImage(image);
    texture->setRepeat(repeatS, repeatT);
    return texture;
}


SgImage* ColladaSceneLoader::Impl::getOrCreateImage(const string& imageId)
{
    auto found = imageMap.find(imageId);
    if(found != imageMap.end()){
        return found->second;
    }
    auto& storedImage = imageMap[imageId]; // keep nullptr for failed images

    auto it = idMap.find(imageId);
    if(it == idMap.end() || strcmp(it->second.name(), "image") != 0){
        warnOnce(formatR(_("Warning: Image \"{0}\" is not found."), imageId));
        return nullptr;
    }
    auto imageElement = it->second;

    string uri;
    if(auto initFrom = imageElement.child("init_from")){
        if(auto ref = initFrom.child("ref")){
            uri = trimmed(ref.child_value()); // COLLADA 1.5
        } else {
            uri = trimmed(initFrom.child_value()); // COLLADA 1.4
        }
    }
    if(uri.empty()){
        warnOnce(
            formatR(_("Warning: Image \"{0}\" does not have a file reference and is skipped."),
                    imageId));
        return nullptr;
    }

    const string decodedUri = removeFileScheme(decodeUri(uri));
    filesystem::path imagePath(fromUTF8(decodedUri));

    // Search for the image file beside the model file and in the image search directories.
    // The bare filename is also tried to support files with invalid absolute paths.
    vector<pair<filesystem::path, filesystem::path>> candidates; // (base directory, relative path)
    if(imagePath.is_absolute()){
        candidates.emplace_back(filesystem::path(), imagePath);
    } else {
        candidates.emplace_back(directory, imagePath);
        for(auto& searchDirectory : imageSearchDirectories){
            candidates.emplace_back(searchDirectory, imagePath);
        }
    }
    if(imagePath.has_parent_path()){
        const auto filename = imagePath.filename();
        candidates.emplace_back(directory, filename);
        for(auto& searchDirectory : imageSearchDirectories){
            candidates.emplace_back(searchDirectory, filename);
        }
    }

    SgImagePtr image = new SgImage;
    bool loaded = false;
    for(auto& [base, relativePath] : candidates){
        auto path = base.empty() ? relativePath : (base / relativePath);
        if(filesystem::exists(path)){
            loaded = imageIO.load(image->image(), toUTF8(path.string()), os());
            if(loaded){
                image->setUriWithFilePathAndBaseDirectory(
                    toUTF8(relativePath.generic_string()), toUTF8(base.string()));
            }
            break;
        }
    }
    if(!loaded){
        os() << formatR(_("Warning: Image file \"{0}\" cannot be loaded."), uri) << endl;
        return nullptr;
    }

    const char* name = imageElement.attribute("name").value();
    if(name[0] != '\0'){
        image->setName(name);
    }

    storedImage = image;
    return storedImage;
}
