/*!
  @author Shin'ichiro Nakaoka
*/

#include "PyUtil.h"
#include "../SceneGraph.h"
#include "../SceneCameras.h"
#include "../CloneMap.h"
#include "../ValueTree.h"

using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

}

namespace cnoid {

void exportPySceneGraph(py::module& m)
{
    py::class_<SgUpdate> sgUpdate(m, "SgUpdate");

    sgUpdate
        .def(py::init<>())
        .def(py::init<int>())
        .def_property("action", &SgUpdate::action, &SgUpdate::setAction)
        .def("setAction", &SgUpdate::setAction)

        // deprecated
        .def("getAction", [](SgUpdate& self){ return self.action(); })
        ;

    py::class_<SgUpdateRef>(m, "SgUpdateRef")
        .def(py::init<>())
        .def(py::init<SgUpdate&>())
        .def(py::init<const SgUpdateRef&>())
        .def(py::init<bool>())
        ;

    py::implicitly_convertible<SgUpdate, SgUpdateRef>();
    py::implicitly_convertible<bool, SgUpdateRef>();
    
    py::enum_<SgUpdate::Action>(sgUpdate, "Action")
        .value("None", SgUpdate::None)
        .value("Added", SgUpdate::Added)
        .value("Removed", SgUpdate::Removed)
        .value("Modified", SgUpdate::Modified)
        // deprecated
        .value("NONE", SgUpdate::Action::NONE)
        .value("ADDED", SgUpdate::Action::ADDED)
        .value("REMOVED", SgUpdate::Action::REMOVED)
        .value("BBOX_UPDATED", SgUpdate::Action::BBOX_UPDATED)
        .value("MODIFIED", SgUpdate::Action::MODIFIED)
        .export_values();

    py::class_<SgObject, SgObjectPtr, Referenced>(m, "SgObject")
        .def_property("name", &SgObject::name, &SgObject::setName)
        .def("setName", &SgObject::setName)
        .def("notifyUpdate",(void(SgObject::*)(SgUpdate&)) &SgObject::notifyUpdate)
        .def("notifyUpdate",(void(SgObject::*)(int)) &SgObject::notifyUpdate, py::arg("action") = SgUpdate::Modified)
        //
        .def("hasUri", &SgObject::hasUri)
        .def("hasAbsoluteUri", &SgObject::hasAbsoluteUri)
        .def("hasUriFragment", &SgObject::hasUriFragment)
        .def("hasUriObjectName", &SgObject::hasUriObjectName)
        .def_property_readonly("uri", &SgObject::uri)
        .def_property_readonly("absoluteUri", &SgObject::absoluteUri)
        .def_property_readonly("localFilePath", &SgObject::localFilePath)
        .def_property_readonly("localFileAbsolutePath", &SgObject::localFileAbsolutePath)
        .def_property_readonly("uriObjectName", &SgObject::uriObjectName)
        .def_property_readonly("uriFragment", &SgObject::uriFragment)
        .def_property_readonly("uriMetadata", &SgObject::uriMetadata)
        //
        .def("setUriWithFilePathAndBaseDirectory", &SgObject::setUriWithFilePathAndBaseDirectory)
        .def("setUriWithFilePathAndCurrentDirectory", &SgObject::setUriWithFilePathAndCurrentDirectory)
        .def("setUri", &SgObject::setUri)
        .def("setUriObjectName", &SgObject::setUriObjectName)
        .def("setUriFragment", &SgObject::setUriFragment)
        .def("setUriMetadata", &SgObject::setUriMetadata)
        .def("clearUri", &SgObject::clearUri)
        // deprecated
        .def("getName", &SgObject::name)
        ;

    py::class_<SgNode, SgNodePtr, SgObject>(m, "SgNode")
        .def(py::init<>())
        .def(py::init<const SgNode&>())
        .def("isGroupNode", &SgNode::isGroupNode)
        .def("boundingBox", [](SgNode &self) { BoundingBox ret = self.boundingBox(); return ret; })
        .def("untransformedBoundingBox", [](SgNode &self) { BoundingBox ret = self.untransformedBoundingBox(); return ret; })
        ;
    
    py::class_<SgGroup, SgGroupPtr, SgNode>(m, "SgGroup")
        .def(py::init<>())
        .def(py::init<const SgGroup&>())
        .def_property_readonly("empty", &SgGroup::empty)
        .def_property_readonly("numChildren", &SgGroup::numChildren)
        .def("clearChildren", &SgGroup::clearChildren, py::arg("update") = false)
        .def_property_readonly("child", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        .def("addChild",
             (void(SgGroup::*)(SgNode*, SgUpdateRef)) &SgGroup::addChild,
             py::arg("node"), py::arg("update") = false)
        .def("contains", &SgGroup::contains)
        .def("findChildIndex", &SgGroup::findChildIndex)
        .def("removeChild", (bool (SgGroup::*)(SgNode*, SgUpdateRef)) &SgGroup::removeChild,
             py::arg("node"), py::arg("update") = false)
        .def("removeChildAt", &SgGroup::removeChildAt)
        // deprecated
        .def("isEmpty", &SgGroup::empty)
        .def("getNumChildren", &SgGroup::numChildren)
        .def("getChild", (SgNode*(SgGroup::*)(int)) &SgGroup::child)
        //
        .def("clone", [](const SgGroup &self) {
            CloneMap map;
            return new SgGroup(self, &map);
        })
        ;
    
    py::class_<SgTransform, SgTransformPtr, SgGroup>(m, "SgTransform");

    py::class_<SgPosTransform, SgPosTransformPtr, SgTransform>(m, "SgPosTransform")
        .def(py::init<>())
        .def(py::init([](Eigen::Ref<const Matrix4RM> T){ return new SgPosTransform(Isometry3(T.matrix())); }))
        .def(py::init<const SgPosTransform&>())
        
        .def_property(
            "T",
            [](SgPosTransform& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](SgPosTransform& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def_property(
            "position",
            [](SgPosTransform& self) -> Isometry3::MatrixType& { return self.T().matrix(); },
            [](SgPosTransform& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def("setPosition",
             [](SgPosTransform& self, Eigen::Ref<const Matrix4RM> T){ self.setPosition(T); })
        .def_property(
            "translation",
            (Isometry3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation,
            [](SgPosTransform& self, Eigen::Ref<const Vector3> p){ self.setTranslation(p); })
        .def("setTranslation",
             [](SgPosTransform& self, Eigen::Ref<const Vector3> p){ self.setTranslation(p); })
        .def_property(
            "rotation",
            (Isometry3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation,
            [](SgPosTransform& self, Eigen::Ref<const Matrix3RM> R){ self.rotation() = R; })
        .def("setRotation",
             [](SgPosTransform& self, Eigen::Ref<const Matrix3RM> R){ self.rotation() = R; })

        // deprecated
        .def("getPosition", [](SgPosTransform& self) -> Isometry3::MatrixType& { return self.T().matrix(); })
        .def("getTranslation", (Isometry3::TranslationPart (SgPosTransform::*)()) &SgPosTransform::translation)
        .def("getRotation", (Isometry3::LinearPart (SgPosTransform::*)()) &SgPosTransform::rotation)
        //
        .def("clone", [](const SgPosTransform &self) {
            CloneMap map;
            return new SgPosTransform(self, &map);
        })
        ;

    py::class_<SgScaleTransform, SgScaleTransformPtr, SgTransform>(m, "SgScaleTransform")
        .def(py::init<>())
        .def(py::init<double>())
        .def(py::init<Vector3 &>())
        .def("setScale", (void (SgScaleTransform::*)(double s)) &SgScaleTransform::setScale)
        .def("setScale", [](SgScaleTransform &self, Vector3 &_in) {  self.setScale(_in); })
        .def_property("scale", (Vector3 & (SgScaleTransform::*)()) &SgScaleTransform::scale, [](SgScaleTransform &self, Vector3 &_in) {  self.setScale(_in); })
        ;

    py::class_<SgPreprocessed, SgPreprocessedPtr, SgNode>(m, "SgPreprocessed")
        ;
    
    py::class_<SgCamera, SgCameraPtr, SgPreprocessed>(m, "SgCamera")
        .def_static(
            "positionLookingFor",
            [](const Vector3& eye, const Vector3& direction, const Vector3& up) -> Isometry3::MatrixType {
                return SgCamera::positionLookingFor(eye, direction, up).matrix();
            })
        .def_static(
            "positionLookingAt",
            [](const Vector3& eye, const Vector3& center, const Vector3& up) -> Isometry3::MatrixType {
                return SgCamera::positionLookingAt(eye, center, up).matrix();
            })
        .def_static("getRight", [](Eigen::Ref<const Matrix4RM> T){ return SgCamera::right(Isometry3(T)); })
        .def_static("getDirection", [](Eigen::Ref<const Matrix4RM> T){ return SgCamera::direction(Isometry3(T)); })
        .def_static("getUp", [](Eigen::Ref<const Matrix4RM> T){ return SgCamera::up(Isometry3(T)); })
        .def_property("nearClipDistance", &SgCamera::nearClipDistance, &SgCamera::setNearClipDistance)
        .def("setNearClipDistance", &SgCamera::setNearClipDistance)
        .def_property("farClipDistance", &SgCamera::farClipDistance, &SgCamera::setFarClipDistance)
        .def("setFarClipDistance", &SgCamera::setFarClipDistance)
        ;

    py::class_<SgPerspectiveCamera, SgPerspectiveCameraPtr, SgCamera>(m, "SgPerspectiveCamera")
        .def(py::init<>())
        .def_property("fieldOfView", &SgPerspectiveCamera::fieldOfView, &SgPerspectiveCamera::setFieldOfView)
        .def("setFieldOfView", &SgPerspectiveCamera::setFieldOfView)
        .def("getFovy", [](SgPerspectiveCamera& self, double aspectRatio){ return self.fovy(aspectRatio); })
        ;

    py::class_<SgOrthographicCamera, SgOrthographicCameraPtr, SgCamera>(m, "SgOrthographicCamera")
        .def(py::init<>())
        .def_property("height", &SgOrthographicCamera::height, &SgOrthographicCamera::setHeight)
        .def("setHeight", &SgOrthographicCamera::setHeight)
        ;

    py::class_<SgSwitch, SgSwitchPtr, SgObject>(m, "SgSwitch")
        .def(py::init<bool>())
        .def_property("turnedOn", &SgSwitch::isTurnedOn, &SgSwitch::setTurnedOn)
        ;

    py::class_<SgSwitchableGroup, SgSwitchableGroupPtr, SgGroup>(m, "SgSwitchableGroup")
        .def(py::init<>())
        .def(py::init<SgSwitch*>())
        .def("setSwitch", &SgSwitchableGroup::setSwitch)
        .def("setTurnedOn", &SgSwitchableGroup::setTurnedOn)
        .def_property("turnedOn", &SgSwitchableGroup::isTurnedOn,
                      [](SgSwitchableGroup &self, bool on) { self.setTurnedOn(on); })
        ;
}

}
