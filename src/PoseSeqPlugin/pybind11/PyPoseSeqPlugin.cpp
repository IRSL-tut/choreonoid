/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PoseSeqItem.h"
#include "../BodyKeyPose.h"
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/PyBase>
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Matrix3RM = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

PYBIND11_MODULE(PoseSeqPlugin, m)
{
    m.doc() = "Choreonoid PoseSeqPlugin module";

    py::module::import("cnoid.BodyPlugin");

    m.def("createPoseSeqItem", [](Item *parent, const std::string &filename) {
        PoseSeqItem *itm = nullptr;
        auto bodyItem = parent->findOwnerItem<BodyItem>(true);
        if(!bodyItem) {
            throw std::runtime_error("PoseSeqItem must be loaded as a child of a BodyItem");
        } else {
            bodyItem->setSelected(true, true);

            itm = new PoseSeqItem;
            itm->clearEditHistory();
            auto pseq = itm->poseSeq();
            bool loaded = pseq->load(filename, bodyItem->body());
            if(!loaded){
                throw std::runtime_error(pseq->errorMessage());
            } else {
                const std::string& name = pseq->name();
                if(!name.empty()){
                    itm->setName(name);
                } else {
                    itm->setName(filename);
                }
                if(pseq->targetBodyName() != bodyItem->body()->name()){
                    // warning
                }
                if (!parent->addChildItem(itm)) {
                    throw std::runtime_error("add item failed");
                }
                itm->notifyUpdate();
            }
        }
        return itm;
    });

    py::class_<PoseSeqItem, PoseSeqItemPtr, Item>(m, "PoseSeqItem")
    .def_property_readonly("poseSeq", &PoseSeqItem::poseSeq)
    .def_property_readonly("targetBodyItem", &PoseSeqItem::targetBodyItem)
    .def_property_readonly("interpolator", &PoseSeqItem::interpolator)
    .def_property_readonly("bodyMotionItem", &PoseSeqItem::bodyMotionItem)
    //.def_property_readonly("", &PoseSeqItem::)
    ;

    py::class_<BodyKeyPose>(m, "BodyKeyPose")
    .def("clear", &BodyKeyPose::clear)
    .def_property_readonly("empty", &BodyKeyPose::empty)
    .def_property_readonly("numJoints", &BodyKeyPose::numJoints)
    .def_property_readonly("numIkLinks", &BodyKeyPose::numIkLinks)
    .def_property_readonly("zmp", &BodyKeyPose::zmp)
    .def("jointDisplacement", &BodyKeyPose::jointDisplacement)
    .def("isJointValid", &BodyKeyPose::isJointValid)
    .def("isJointStationaryPoint", &BodyKeyPose::isJointStationaryPoint)
    .def("isZmpValid", &BodyKeyPose::isZmpValid)
    .def("isZmpStationaryPoint", &BodyKeyPose::isZmpStationaryPoint)
    ;

    py::class_<BodyKeyPose::LinkInfo>(m, "LinkInfo")
    .def("isBaseLink", &BodyKeyPose::LinkInfo::isBaseLink)
    .def("isStationaryPoint", &BodyKeyPose::LinkInfo::isStationaryPoint)
    .def("isTouching", &BodyKeyPose::LinkInfo::isTouching)
    .def("isSlave", &BodyKeyPose::LinkInfo::isSlave)
    .def_property_readonly("T", [](BodyKeyPose::LinkInfo &self) -> Isometry3::MatrixType& {
        return self.T().matrix();
    })
    .def_property_readonly("partingDirection", &BodyKeyPose::LinkInfo::partingDirection)
    .def_property_readonly("contactPoints", &BodyKeyPose::LinkInfo::contactPoints)
    ;

    py::class_<SequentialPose>(m, "SequentialPose")
    .def_property_readonly("time", &SequentialPose::time)
    .def_property_readonly("pose", [](SequentialPose &self) {
        AbstractPose* pose = self.pose();
        return dynamic_cast<BodyKeyPose *>(pose);
    })
    ;

    py::class_<PoseSeq, PoseSeqPtr>(m, "PoseSeq")
    .def_property_readonly("name", &PoseSeq::name)
    .def_property_readonly("empty", &PoseSeq::empty)
    .def_property_readonly("size", &PoseSeq::size)
    .def("poses", [](PoseSeq &self) {
        py::list res;
        for(auto it = self.begin(); it != self.end(); it++) {
            res.append(*it);
        }
        return res;
    })
    ;
    //
    py::class_<PoseSeqInterpolator, PoseSeqInterpolatorPtr>(m, "PoseSeqInterpolator")
    .def("body", &PoseSeqInterpolator::body)
    .def_property_readonly("beginningTime", &PoseSeqInterpolator::beginningTime)
    .def_property_readonly("endingTime", &PoseSeqInterpolator::endingTime)
    .def("interpolate", (bool (PoseSeqInterpolator::*)(double))&PoseSeqInterpolator::interpolate)
    .def("seek", (bool (PoseSeqInterpolator::*)(double))&PoseSeqInterpolator::seek)
    .def_property_readonly("baseLinkIndex", &PoseSeqInterpolator::baseLinkIndex)
    .def_property_readonly("baseLinkPosition", [](PoseSeqInterpolator &self)  {
        Isometry3 res;
        self.getBaseLinkPosition(res);
        return res.matrix();
    })
    .def("jointPosition", &PoseSeqInterpolator::jointPosition)
    .def_property_readonly("ZMP", &PoseSeqInterpolator::ZMP)
    .def("getJointDisplacements", [](PoseSeqInterpolator &self) {
        std::vector<stdx::optional<double>> res;
        self.getJointDisplacements(res);
        return res;
    })
    .def("update", &PoseSeqInterpolator::update)
    ;
}
