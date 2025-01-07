#include "PyUtil.h"
#include "../SceneDrawables.h"
#include "../CloneMap.h"

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> MatrixfRM;
typedef Eigen::Ref<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> RefMatrixfRM;

typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4RM;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Matrix3RM;

void setTextureImage(SgShape &shape, const std::string &name)
{
    SgTexture *tex = new SgTexture();
    SgImage *sgimg = tex->getOrCreateImage();
    sgimg->setUri(name, name);
    bool res = sgimg->image().load(name);
    if (res) {
        shape.setTexture(tex);
    } else {
        delete tex;
    }
}

void exportPySceneDrawables(py::module& m)
{
    py::class_<SgMaterial, SgMaterialPtr, SgObject>(m, "SgMaterial")
        .def(py::init<>())
        .def_property("ambientIntensity", &SgMaterial::ambientIntensity, &SgMaterial::setAmbientIntensity)
        .def("setAmbientIntensity", &SgMaterial::setAmbientIntensity)
        .def_property("diffuseColor",
                      &SgMaterial::diffuseColor,
                      [](SgMaterial& self, const Vector3f &c){ self.setDiffuseColor(c); })
        .def("setDiffuseColor", [](SgMaterial& self, const Vector3f &c){ self.setDiffuseColor(c); })
        .def_property("emissiveColor",
                      &SgMaterial::emissiveColor,
                      [](SgMaterial& self, const Vector3f &c){ self.setEmissiveColor(c); })
        .def("setEmissiveColor", [](SgMaterial& self, const Vector3f &c){ self.setEmissiveColor(c); })
        .def_property("specularExponent", &SgMaterial::specularExponent, &SgMaterial::specularExponent)
        .def("setSpecularExponent", &SgMaterial::setSpecularExponent)
        .def_property("specularColor",
                      &SgMaterial::specularColor,
                      [](SgMaterial& self, const Vector3f &c){ self.setSpecularColor(c); })
        .def("setSpecularColor", [](SgMaterial& self, const Vector3f &c){ self.setSpecularColor(c); })
        .def_property("transparency", &SgMaterial::transparency, &SgMaterial::setTransparency)
        .def("setTransparency", &SgMaterial::setTransparency)
        ;
// Comment for implementation of method
// <vectorArray> xx, xxs
// hasXXs     bool()
// setXXs     void(Matrix)
// xxs        property_ro Matrix()
// sizeOfxxs  property_ro int()
// appendXX   void(Vec)
// xx         Vec(int)
// setXX      void(int, Vec)
// 
// <Indices> XX
// hasXXIndices bool()
// setXXIndices void(vector<int>)
// xxIndices property_ro vector<int>()
// sizeOfXXIndices prop_ro int()
// resizeXXIndices void(int)
// getXXIndex int(int)
// setXXIndex void(int, int)
    py::class_<SgMeshBase, SgMeshBasePtr, SgObject>(m, "SgMeshBase")
        .def("boundingBox", [](SgMeshBase &self) { BoundingBox ret = self.boundingBox(); return ret; })
        .def("updateBoundingBox", &SgMeshBase::updateBoundingBox)
        // <VectorArray> vertex, vertices
        .def("hasVertices", &SgMeshBase::hasVertices)
        .def("setVertices", [](SgMeshBase &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgVertexArray *va_ = self.getOrCreateVertices(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("vertices", [](SgMeshBase &self) {
            SgVertexArray *va_ = self.vertices();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfVertices", [](SgMeshBase &self) { if (!self.vertices()) return (std::size_t)0; return self.vertices()->size(); })
        .def("appendVertex", [](SgMeshBase &self, const Vector3f &v) { self.vertices()->push_back(v); })
        .def("vertex", [](SgMeshBase &self, int idx) { return self.vertices()->at(idx); })
        .def("setVertex", [](SgMeshBase &self, int idx, const Vector3f &v) { self.vertices()->at(idx) = v; })
        // <VectorArray> normal, normals
        .def("hasNormals", &SgMeshBase::hasNormals)
        .def("setNormals", [](SgMeshBase &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgNormalArray *va_ = self.getOrCreateNormals();
            va_->resize(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("normals",  [](SgMeshBase &self) {
            SgNormalArray *va_ = self.normals();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfNormals", [](SgMeshBase &self) { if (!self.normals()) return (std::size_t)0; return self.normals()->size(); })
        .def("appendNormal", [](SgMeshBase &self, const Vector3f &v) { self.normals()->push_back(v); })
        .def("normal", [](SgMeshBase &self, int idx) { return self.normals()->at(idx); })
        .def("setNormal", [](SgMeshBase &self, int idx, const Vector3f &v) { self.normals()->at(idx) = v; })
        // <VectorArray> color, colors
        .def("hasColors", &SgMeshBase::hasColors)
        .def("setColors", [](SgMeshBase &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgColorArray *va_ = self.getOrCreateColors(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("colors",  [](SgMeshBase &self) {
            SgColorArray *va_ = self.colors();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfColors", [](SgMeshBase &self) { if(!self.colors()) return (std::size_t)0;  return self.colors()->size(); })
        .def("appendColor", [](SgMeshBase &self, const Vector3f &v) { self.colors()->push_back(v); })
        .def("color", [](SgMeshBase &self, int idx) { return self.colors()->at(idx); })
        .def("setColor", [](SgMeshBase &self, int idx, const Vector3f &v) { self.colors()->at(idx) = v; })
        // <VectorArray> texcoord, texcoords
        .def("hasTexCoords", &SgMeshBase::hasTexCoords)
        .def("setTexCoords", [](SgMeshBase &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgTexCoordArray *va_ = self.getOrCreateTexCoords();
            va_->resize(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("texCoords",  [](SgMeshBase &self) {
            SgTexCoordArray *va_ = self.texCoords();
            if (!va_) { MatrixfRM mat(0, 2); return mat; }
            MatrixfRM mat(va_->size(), 2);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfTexCoords", [](SgMeshBase &self) { if(!self.texCoords()) return (std::size_t)0;  return self.texCoords()->size(); })
        .def("appendTexCoord", [](SgMeshBase &self, const Vector2f &v) { self.texCoords()->push_back(v); })
        .def("texCoord", [](SgMeshBase &self, int idx) { return self.texCoords()->at(idx); })
        .def("setTexCoord", [](SgMeshBase &self, int idx, const Vector2f &v) { self.texCoords()->at(idx) = v; })
        // <Indices> faceVertex
        .def("hasFaceVertexIndices", &SgMeshBase::hasFaceVertexIndices)
        .def("setFaceVertexIndices", [](SgMeshBase &self, std::vector<int> &_ind) { self.faceVertexIndices() = _ind; })
        .def_property_readonly("faceVertexIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::faceVertexIndices)
        .def_property_readonly("sizeOfFaceVertexIndices", [](SgMeshBase &self){ return self.faceVertexIndices().size(); })
        .def("resizeFaceVertexindices", [](SgMeshBase &self, int n) { self.faceVertexIndices().resize(n); })
        .def("getFaceVertexindex", [](SgMeshBase &self, int idx) { return self.faceVertexIndices().at(idx); })
        .def("setFaceVertexindex", [](SgMeshBase &self, int idx, int fceidx) { self.faceVertexIndices().at(idx) = fceidx; })
        // <Indices> normal
        .def("hasNormalIndices", &SgMeshBase::hasNormalIndices)
        .def("setNormalIndices", [](SgMeshBase &self, std::vector<int> &_ind) { self.normalIndices() = _ind; })
        .def_property_readonly("normalIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::normalIndices)
        .def_property_readonly("sizeOfNormalIndices", [](SgMeshBase &self){ return self.normalIndices().size(); })
        .def("resizeNormalindices", [](SgMeshBase &self, int n) { self.normalIndices().resize(n); })
        .def("getNormalindex", [](SgMeshBase &self, int idx) { return self.normalIndices().at(idx); })
        .def("setNormalindex", [](SgMeshBase &self, int idx, int nmidx) { self.normalIndices().at(idx) = nmidx; })
        // <Indices> color
        .def("hasColorIndices", &SgMeshBase::hasColorIndices)
        .def("setColorIndices", [](SgMeshBase &self, std::vector<int> &_ind) { self.colorIndices() = _ind; })
        .def_property_readonly("colorIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::colorIndices)
        .def_property_readonly("sizeOfColorIndices", [](SgMeshBase &self){ return self.colorIndices().size(); })
        .def("resizeColorindices", [](SgMeshBase &self, int n) { self.colorIndices().resize(n); })
        .def("getColorindex", [](SgMeshBase &self, int idx) { return self.colorIndices().at(idx); })
        .def("setColorindex", [](SgMeshBase &self, int idx, int colidx) { self.colorIndices().at(idx) = colidx; })
        // <Indices> texcoord
        .def("hasTexCoordIndices", &SgMeshBase::hasTexCoordIndices)
        .def("setTexCoordIndices", [](SgMeshBase &self, std::vector<int> &_ind) { self.texCoordIndices() = _ind; })
        .def_property_readonly("texCoordIndices", (SgIndexArray&(SgMeshBase::*)())&SgMeshBase::texCoordIndices)
        .def_property_readonly("sizeOfTexCoordIndices", [](SgMeshBase &self){ return self.texCoordIndices().size(); })
        .def("resizeTexCoordindices", [](SgMeshBase &self, int n) { self.texCoordIndices().resize(n); })
        .def("getTexCoordindex", [](SgMeshBase &self, int idx) { return self.texCoordIndices().at(idx); })
        .def("setTexCoordindex", [](SgMeshBase &self, int idx, int txcidx) { self.texCoordIndices().at(idx) = txcidx; })
        //
        .def("creaseAngle", &SgMeshBase::creaseAngle)
        .def("isSolid", &SgMeshBase::isSolid)
        ;

    py::class_<SgMesh, SgMeshPtr, SgMeshBase> sgMesh(m, "SgMesh");

    py::enum_<SgMesh::PrimitiveType>(sgMesh, "PrimitiveType")
        .value("MeshType", SgMesh::MeshType)
        .value("BoxType", SgMesh::BoxType)
        .value("SphereType", SgMesh::SphereType)
        .value("CylinderType", SgMesh::CylinderType)
        .value("ConeType", SgMesh::ConeType)
        .value("CapsuleType", SgMesh::CapsuleType)
        .export_values();

    py::enum_<SgMesh::ExtraDivisionMode>(sgMesh, "ExtraDivisionMode")
        .value("ExtraDivisionPreferred", SgMesh::ExtraDivisionPreferred)
        .value("ExtraDivisionX", SgMesh::ExtraDivisionX)
        .value("ExtraDivisionY", SgMesh::ExtraDivisionY)
        .value("ExtraDivisionZ", SgMesh::ExtraDivisionZ)
        .value("ExtraDivisionAll", SgMesh::ExtraDivisionAll)
        .export_values();

    sgMesh
        .def(py::init<>())
        .def("updateBoundingBox", &SgMesh::updateBoundingBox)
        // <Indices> triangle == faceVertexIndices
        .def("triangleVertices", (SgIndexArray&(SgMesh::*)())&SgMesh::triangleVertices)
        //.def("triangleVertices", [](SgMesh &self) { SgIndexArray idx_ = self.triangleVertices(); return idx_; })
        .def("hasTriangles", &SgMesh::hasTriangles)
        .def("numTriangles", &SgMesh::numTriangles)// TODO property
        .def("triangle", [](SgMesh &self, int index) { Array3i ret = self.triangle(index); return ret; })
        .def("setTriangle", &SgMesh::setTriangle)
        .def("addTriangle", (void(SgMesh::*)(int, int, int))&SgMesh::addTriangle)
        .def("setNumTriangles", &SgMesh::setNumTriangles)
        .def("clearTriangles", &SgMesh::clearTriangles)
        .def("transform", [](SgMesh &self, Eigen::Ref<Matrix4RM> T) {
                              Isometry3 iso(T); self.transform(iso); })
        .def("translate", [](SgMesh &self, const Vector3 &trs) {
                              Vector3f tf = trs.cast<float>(); self.translate(tf); })
        .def("rotate", [](SgMesh &self, Eigen::Ref<Matrix3RM> rot) {
                           Matrix3f rf(rot.cast<float>()); self.rotate(rf); })
        .def_property("divisionNumber", &SgMesh::divisionNumber, &SgMesh::setDivisionNumber)
        .def_property("extraDivisionNumber", &SgMesh::extraDivisionNumber, &SgMesh::setExtraDivisionNumber)
        .def_property("extraDivisionMode", &SgMesh::extraDivisionMode, &SgMesh::setExtraDivisionMode)
        .def("setMeshType", [] (SgMesh &self) {  self.setPrimitive(SgMesh::Mesh()); })
        .def_property_readonly("primitiveType", &SgMesh::primitiveType)
        .def_property_readonly("primitive", [](SgMesh  &self) {
            int tp = self.primitiveType();
            if (tp == 1) {
                SgMesh::Box *pri = new SgMesh::Box(self.primitive<SgMesh::Box>());
                return py::cast(pri);
            } else if (tp == 2) {
                SgMesh::Sphere *pri = new SgMesh::Sphere(self.primitive<SgMesh::Sphere>());
                return py::cast(pri);
            } else if (tp == 3) {
                SgMesh::Cylinder *pri = new SgMesh::Cylinder(self.primitive<SgMesh::Cylinder>());
                return py::cast(pri);
            } else if (tp == 4) {
                SgMesh::Cone *pri = new SgMesh::Cone(self.primitive<SgMesh::Cone>());
                return py::cast(pri);
            } else if (tp == 5) {
                SgMesh::Capsule *pri = new SgMesh::Capsule(self.primitive<SgMesh::Capsule>());
                return py::cast(pri);
            }
            return py::cast(nullptr); })
        ;

    //// PrimitiveTypes
    py::class_<SgMesh::Box> (m, "SgMeshBox")
        .def_property_readonly("volume", [](SgMesh::Box &self) { return self.size[0] * self.size[1] * self.size[2]; })
        .def_property_readonly("COM", [](SgMesh::Box &self) { return Vector3::Zero(); })
        .def_property_readonly("inertia", [](SgMesh::Box &self) { Matrix3 ret = Matrix3::Zero();
                ret(0, 0) = (self.size[1] * self.size[1] + self.size[2] * self.size[2] ) / 12.0;
                ret(1, 1) = (self.size[0] * self.size[0] + self.size[2] * self.size[2] ) / 12.0;
                ret(2, 2) = (self.size[0] * self.size[0] + self.size[1] * self.size[1] ) / 12.0;
                return ret; })
        .def_property_readonly("size", [](SgMesh::Box &self) { return self.size; })
        ;
    py::class_<SgMesh::Sphere> (m, "SgMeshSphere")
        .def_property_readonly("volume", [](SgMesh::Sphere &self) { return 4.0 * M_PI * self.radius * self.radius * self.radius / 3.0; })
        .def_property_readonly("COM", [](SgMesh::Sphere &self) { return Vector3::Zero(); })
        .def_property_readonly("inertia", [](SgMesh::Sphere &self) { Matrix3 ret = Matrix3::Zero();
                ret(0, 0) = ret(1, 1) = ret(2, 2) = 2.0 * (self.radius * self.radius) / 5.0;
                return ret; })
        .def_property_readonly("radius", [](SgMesh::Sphere &self) { return self.radius; })
        ;
    py::class_<SgMesh::Cylinder> (m, "SgMeshCylinder") // bottom/top circle on xz-plane / height: y-direction
        .def_property_readonly("volume", [](SgMesh::Cylinder &self) { return M_PI * self.radius * self.radius * self.height; })
        .def_property_readonly("COM", [](SgMesh::Cylinder &self) { return Vector3::Zero(); })
        .def_property_readonly("inertia", [](SgMesh::Cylinder &self) { Matrix3 ret = Matrix3::Zero();
                ret(1, 1) = (self.radius * self.radius) / 2.0;
                ret(0, 0) = ret(2, 2) = ((self.radius * self.radius) / 4.0) + ((self.height * self.height) / 12.0);
                return ret; })
        .def_property_readonly("radius", [](SgMesh::Cylinder &self) { return self.radius; })
        .def_property_readonly("height", [](SgMesh::Cylinder &self) { return self.height; })
        ;
    py::class_<SgMesh::Cone> (m, "SgMeshCone") // bottom circle on xz-plane (y = -height/2) / height: y-direction
        .def_property_readonly("volume", [](SgMesh::Cone &self) { return M_PI * self.radius * self.radius * self.height / 3.0; })
        .def_property_readonly("COM", [](SgMesh::Cone &self) { Vector3 ret = Vector3::Zero(); ret[1] = - self.height/2.0; return ret; })
        .def_property_readonly("inertia", [](SgMesh::Cone &self) { Matrix3 ret = Matrix3::Zero();
                ret(1, 1) = 0.3 * (self.radius * self.radius);
                ret(0, 0) = ret(2, 2) = 0.15 * (self.radius * self.radius) + 3.0 / 80.0 * (self.height * self.height);
                return ret; })
        .def_property_readonly("radius", [](SgMesh::Cone &self) { return self.radius; })
        .def_property_readonly("height", [](SgMesh::Cone &self) { return self.height; })
        ;
    py::class_<SgMesh::Capsule> (m, "SgMeshCapsule") // circle on xz-plane / height: y-direction
        .def_property_readonly("volume", [](SgMesh::Capsule &self) {
            // return (4.0 * M_PI * self.radius * self.radius * self.radius / 3.0) + (M_PI * self.radius * self.radius * self.height);
            return M_PI * self.radius * self.radius * ((4.0 * self.radius / 3.0) + self.height); })
        .def_property_readonly("COM", [](SgMesh::Capsule &self) { return Vector3::Zero(); })
        .def_property_readonly("inertia", [](SgMesh::Capsule &self) { Matrix3 ret = Matrix3::Zero();
                double ratio_cyl = self.height / ((4.0 * self.radius / 3.0) + self.height);
                double ratio_sph = 1 - ratio_cyl;
                double sph_i = ratio_sph * 2.0 * (self.radius * self.radius) / 5.0;
                ret(1, 1) = (ratio_cyl * (self.radius * self.radius) / 2.0) + sph_i;
                ret(0, 0) = ret(2, 2) = ratio_cyl * (((self.radius * self.radius) / 4.0) + ((self.height * self.height) / 12.0))
                                        + sph_i + ratio_sph * self.height * self.height / 4.0;
                return ret; })
        .def_property_readonly("radius", [](SgMesh::Capsule &self) { return self.radius; })
        .def_property_readonly("height", [](SgMesh::Capsule &self) { return self.height; })
        ;

    py::class_<SgShape, SgShapePtr, SgNode>(m, "SgShape")
        .def(py::init<>())
        .def_property("mesh", (SgMesh* (SgShape::*)()) &SgShape::mesh, &SgShape::setMesh)
        .def("setMesh", &SgShape::setMesh)
        .def("getOrCreateMesh", &SgShape::getOrCreateMesh)
        .def_property("material", (SgMaterial* (SgShape::*)()) &SgShape::material, &SgShape::setMaterial)
        .def("setMaterial", &SgShape::setMaterial)
        .def("getOrCreateMaterial", &SgShape::getOrCreateMaterial)
        .def("setTextureImage", [](SgShape &self, const std::string &name) { setTextureImage(self, name); })
        ;

    py::class_<SgPlot, SgPlotPtr, SgNode>(m, "SgPlot")
        .def_property("material", (SgMaterial* (SgPlot::*)()) &SgPlot::material, &SgPlot::setMaterial)
        .def("setMaterial", &SgPlot::setMaterial)
        .def("updateBoundingBox", &SgPlot::updateBoundingBox)
        .def("getOrCreateMaterial", &SgPlot::getOrCreateMaterial)
        // <VectorArray> vertex, vertices
        .def("hasVertices", &SgPlot::hasVertices)
        .def("setVertices", [](SgPlot &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgVertexArray *va_ = self.getOrCreateVertices(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("vertices",  [](SgPlot &self) {
            SgVertexArray *va_ = self.vertices();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfVertices", [](SgPlot &self) { if (!self.vertices()) return (std::size_t)0; return self.vertices()->size(); })
        .def("appendVertex", [](SgPlot &self, const Vector3f &v) { self.vertices()->push_back(v); })
        .def("vertex", [](SgPlot &self, int idx) { return self.vertices()->at(idx); })
        .def("setVertex", [](SgPlot &self, int idx, const Vector3f &v) { self.vertices()->at(idx) = v; })
        // <VectorArray> color, colors
        .def("hasColors", &SgPlot::hasColors)
        .def("setColors", [](SgPlot &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgVertexArray *va_ = self.getOrCreateColors(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("colors",  [](SgPlot &self) {
            SgVertexArray *va_ = self.colors();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfColors", [](SgPlot &self) { if(!self.colors()) return (std::size_t)0;  return self.colors()->size(); })
        .def("appendColor", [](SgPlot &self, const Vector3f &v) { self.colors()->push_back(v); })
        .def("color", [](SgPlot &self, int idx) { return self.colors()->at(idx); })
        .def("setColor", [](SgPlot &self, int idx, const Vector3f &v) { self.colors()->at(idx) = v; })
        // <Indices> color
        .def("hasColorIndices", [](SgPlot &self) { return !self.colorIndices().empty(); })
        .def("setColorIndices", [](SgPlot &self, std::vector<int> &_ind) { self.colorIndices() = _ind; })
        .def_property_readonly("colorIndices", (SgIndexArray&(SgPlot::*)())&SgPlot::colorIndices)
        .def_property_readonly("sizeOfColorIndices", [](SgPlot &self){ return self.colorIndices().size(); })
        .def("resizeColorIndices", [](SgPlot &self, int n) { self.colorIndices().resize(n); })
        .def("getColorIndex", [](SgPlot &self, int idx) { return self.colorIndices().at(idx); })
        .def("setColorIndex", [](SgPlot &self, int idx, int colidx) { self.colorIndices().at(idx) = colidx; })
         // <VectorArray> normal, normals
        .def("hasNormals", &SgPlot::hasNormals)
        .def("setNormals", [](SgPlot &self, RefMatrixfRM mat) {
            // mat.cols() == 3
            int size = mat.rows();
            SgVertexArray *va_ = self.getOrCreateNormals();
            va_->resize(size);
            for(int i = 0; i < size; i++) va_->at(i) = mat.row(i);  })
        .def_property_readonly("normals",  [](SgPlot &self) {
            SgVertexArray *va_ = self.normals();
            if (!va_) { MatrixfRM mat(0, 3); return mat; }
            MatrixfRM mat(va_->size(), 3);
            for(int i = 0; i < va_->size(); i ++) mat.row(i) = va_->at(i);
            return mat; })
        .def_property_readonly("sizeOfNormals", [](SgPlot &self) { if (!self.normals()) return (std::size_t)0; return self.normals()->size(); })
        .def("appendNormal", [](SgPlot &self, const Vector3f &v) { self.normals()->push_back(v); })
        .def("normal", [](SgPlot &self, int idx) { return self.normals()->at(idx); })
        .def("setNormal", [](SgPlot &self, int idx, const Vector3f &v) { self.normals()->at(idx) = v; })
         // <Indices> normal
        .def("hasNormalIndices", [](SgPlot &self) { return !self.normalIndices().empty(); })
        .def("setNormalIndices", [](SgPlot &self, std::vector<int> &_ind) { self.normalIndices() = _ind; })
        .def_property_readonly("normalIndices", (SgIndexArray&(SgPlot::*)())&SgPlot::normalIndices)
        .def_property_readonly("sizeOfNormalIndices", [](SgPlot &self){ return self.normalIndices().size(); })
        .def("resizeNormalIndices", [](SgPlot &self, int n) { self.normalIndices().resize(n); })
        .def("getNormalIndex", [](SgPlot &self, int idx) { return self.normalIndices().at(idx); })
        .def("setNormalIndex", [](SgPlot &self, int idx, int nomidx) { self.normalIndices().at(idx) = nomidx; })
        ;

    py::class_<SgPointSet, SgPointSetPtr, SgPlot>(m, "SgPointSet")
        .def(py::init<>())
        .def_property("pointSize", &SgPointSet::pointSize, &SgPointSet::setPointSize)
        ;

    py::class_<SgLineSet, SgLineSetPtr, SgPlot>(m, "SgLineSet")
        .def(py::init<>())
        // <Indices> lineVertex
        .def("setLineVertexIndices", [](SgLineSet &self, std::vector<int> &_ind) { self.lineVertexIndices() = _ind; })
        .def_property("lineVertexIndices", (SgIndexArray&(SgLineSet::*)())&SgLineSet::lineVertexIndices,
                      [](SgLineSet &self, std::vector<int> &_ind) { self.lineVertexIndices() = _ind; })
        .def_property("numLines", &SgLineSet::numLines, &SgLineSet::setNumLines)
        .def_property("lineWidth", &SgLineSet::lineWidth, &SgLineSet::setLineWidth)
        .def("reserveNumLines", &SgLineSet::reserveNumLines)
        .def("clearLines", &SgLineSet::reserveNumLines)
        .def("line", [](SgLineSet &self, int index) {
            SgLineSet::LineRef r = self.line(index);
            std::vector<int> res(2); res[0] = r[0]; res[1] = r[1];
            return res; })
        .def("setLine", &SgLineSet::setLine)
        .def("addLine", (void (SgLineSet::*)(int, int))&SgLineSet::addLine)
        .def("addLine", [](SgLineSet &self, std::vector<int> &in) {
            if (in.size() > 1) self.addLine(in[0], in[1]); })
        .def("resizeColorIndicesForNumLines", &SgLineSet::resizeColorIndicesForNumLines)
        .def("setLineColor", &SgLineSet::setLineColor)
        ;

    py::class_<SgText, SgTextPtr, SgNode>(m, "SgText")
        .def(py::init<>())
        .def_property("text", &SgText::text, &SgText::setText)
        .def_property("textHeight", &SgText::textHeight, &SgText::setTextHeight)
        .def_property("color", &SgText::color, [](SgText &self, Vector3f &_in) { self.setColor(_in); })
        ;
}

}
