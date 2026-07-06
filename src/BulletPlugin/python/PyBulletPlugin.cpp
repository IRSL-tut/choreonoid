#include "../BulletSimulatorItem.h"
#include <cnoid/PyBase>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(BulletPlugin, m)
{
    m.doc() = "Choreonoid BulletPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<BulletSimulatorItem, SimulatorItem>(m, "BulletSimulatorItem")
        .def(nb::init<>())
        .def("setGravity",
             [](BulletSimulatorItem& self, const python::Vector3Arg& g){ self.setGravity(g.value); })
        .def_prop_ro("gravity", &BulletSimulatorItem::gravity)
        ;

    PyItemList<BulletSimulatorItem>(m, "BulletSimulatorItemList");
}
