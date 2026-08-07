#include "../App.h"
#include "../AppUtil.h"
#include <cnoid/PyUtil>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyAppUtil(nb::module_ m)
{
    nb::class_<AppUtil::OngoingProcess, Referenced>(m, "OngoingProcess")
        .def_prop_ro("name", &AppUtil::OngoingProcess::name)
        .def("finish", &AppUtil::OngoingProcess::finish)
        ;

    nb::class_<App>(m, "App")
        .def_static("updateGui", &AppUtil::updateGui, nb::arg("allEvents") = false)
        .def_static("exit", &App::exit, nb::arg("returnCode") = 0)
        .def_prop_ro_static("sigAboutToQuit", [](nb::handle){ return AppUtil::sigAboutToQuit(); })
        .def_prop_ro_static("isNonInteractiveMode", [](nb::handle){ return AppUtil::isNonInteractiveMode(); })
        .def_prop_ro_static("isBatchMode", [](nb::handle){ return AppUtil::isBatchMode(); })
        .def_static("beginOngoingProcess", &AppUtil::beginOngoingProcess, nb::arg("name"))
        ;
}

}
