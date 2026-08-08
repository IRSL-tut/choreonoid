#include "PyUtil.h"
#include "../MessageOut.h"

namespace nb = nanobind;
using namespace cnoid;

namespace cnoid {

void exportPyMessageOut(nb::module_& m)
{
    nb::class_<MessageOut, Referenced> messageOut(m, "MessageOut");

    nb::enum_<MessageOut::MessageType>(messageOut, "MessageType", nb::is_arithmetic())
        .value("Normal", MessageOut::Normal)
        .value("Highlighted", MessageOut::Highlighted)
        .value("Warning", MessageOut::Warning)
        .value("Error", MessageOut::Error)
        .export_values();

    messageOut
        .def_prop_ro_static("master", [](nb::handle){ return MessageOut::master(); })
        .def_prop_ro_static("interactive", [](nb::handle){ return MessageOut::interactive(); })
        .def("put", (void(MessageOut::*)(const std::string&, int))&MessageOut::put,
             nb::arg("message"), nb::arg("type") = (int)MessageOut::Normal)
        .def("putln", (void(MessageOut::*)(const std::string&, int))&MessageOut::putln,
             nb::arg("message"), nb::arg("type") = (int)MessageOut::Normal)
        .def("notify", (void(MessageOut::*)(const std::string&, int))&MessageOut::notify,
             nb::arg("message"), nb::arg("type") = (int)MessageOut::Normal)
        .def("putHighlighted", &MessageOut::putHighlighted)
        .def("putHighlightedln", &MessageOut::putHighlightedln)
        .def("putWarning", &MessageOut::putWarning)
        .def("putWarningln", &MessageOut::putWarningln)
        .def("putError", &MessageOut::putError)
        .def("putErrorln", &MessageOut::putErrorln)
        .def("flush", &MessageOut::flush)
        ;
}

}
