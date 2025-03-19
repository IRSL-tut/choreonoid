/*!
  @author Shin'ichiro Nakaoka
 */

#include "PyDeviceList.h"
#include "../Device.h"
#include "../Link.h"
#include "../ForceSensor.h"
#include "../AccelerationSensor.h"
#include "../RateGyroSensor.h"
#include "../Imu.h"
#include "../PointLight.h"
#include "../SpotLight.h"
#include "../MarkerDevice.h"
#include "../ZmpDevice.h"
#include "../Camera.h"
#include "../RangeSensor.h"
#include <cnoid/PyUtil>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace {

using Matrix4RM = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

}

namespace cnoid {

void exportPyDeviceTypes(py::module& m)
{
    py::class_<Device, DevicePtr, Referenced>(m, "Device")
        .def("__repr__",
             [](const Device &self) {
                 return string("<cnoid.Body.") + self.typeName() + " named '" + self.name() + "'>"; })
        .def_property_readonly("typeName", &Device::typeName)
        .def_property("index", &Device::index, &Device::setIndex)
        .def("setIndex", &Device::setIndex)
        .def_property("id", &Device::id, &Device::setId)
        .def("setId", &Device::setId)
        .def_property("name", &Device::name, &Device::setName)
        .def("setName", &Device::setName)
        .def("link", (Link*(Device::*)())&Device::link)
        .def("setLink", &Device::setLink)
        .def("clone", (Device*(Device::*)()const) &Device::clone)
        .def("clearState", &Device::clearState)
        .def("hasStateOnly", &Device::hasStateOnly)
        .def_property(
            "T_local",
            [](Device& self) -> Isometry3::MatrixType& { return self.T_local().matrix(); },
            [](Device& self, Eigen::Ref<const Matrix4RM> T) { self.T_local() = T; })

        // deprecated
        .def("getIndex", &Device::index)
        .def("getId", &Device::id)
        .def("getName", &Device::name)
        .def("getLink", (Link*(Device::*)())&Device::link)
        ;

    py::class_<ForceSensor, ForceSensorPtr, Device>(m, "ForceSensor")
    .def(py::init<>())
    .def_property_readonly("f", [](const ForceSensor &self) { return self.f(); })
    .def_property_readonly("tau", [](const ForceSensor &self) { return self.tau(); })
    .def_property_readonly("wrench", [](const ForceSensor &self) { return self.wrench(); })
    ;
    py::class_<AccelerationSensor, AccelerationSensorPtr, Device>(m, "AccelerationSensor")
    .def(py::init<>())
    .def_property_readonly("dv", [](const AccelerationSensor &self) { return self.dv(); })
    ;
    py::class_<RateGyroSensor, RateGyroSensorPtr, Device>(m, "RateGyroSensor")
    .def(py::init<>())
    .def_property_readonly("w", [](const RateGyroSensor &self) { return self.w(); })
    ;
    py::class_<Imu, ImuPtr, Device>(m, "Imu")
    .def(py::init<>())
    .def_property_readonly("w", [](const Imu &self) { return self.w(); })
    .def_property_readonly("dv", [](const Imu &self) { return self.dv(); })
    ;

    py::class_<Light, LightPtr, Device>(m, "Light")
    ;
    py::class_<PointLight, PointLightPtr, Light>(m, "PointLight")
    .def(py::init<>())
    ;
    py::class_<SpotLight, SpotLightPtr, Light>(m, "SpotLight")
    .def(py::init<>())
    ;
    py::class_<MarkerDevice, MarkerDevicePtr, Device>(m, "MarkerDevice")
    .def(py::init<>())
    ;
    py::class_<ZmpDevice, ZmpDevicePtr, Device>(m, "ZmpDevice")
    .def(py::init<>())
    ;
    py::class_<VisionSensor, VisionSensorPtr, Device>(m, "VisionSensor")
    ;
    py::class_<Camera, CameraPtr, VisionSensor>(m, "Camera")
    .def(py::init<>())
    ;
    py::class_<RangeSensor, RangeSensorPtr, VisionSensor>(m, "RangeSensor")
    .def(py::init<>())
    ;

    PyDeviceList<Device>(m, "DeviceList");
    PyDeviceList<ForceSensor>(m, "ForceSensorList");
}

}
