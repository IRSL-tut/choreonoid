#include "PyUtil.h"
#include "../MessageOut.h"

namespace py = pybind11;
using namespace cnoid;

namespace cnoid {

void exportPyMessageOut(py::module& m)
{
    py::class_<MessageOut, MessageOutPtr, Referenced> messageOut(m, "MessageOut");

    py::enum_<MessageOut::MessageType>(messageOut, "MessageType")
        .value("Normal", MessageOut::Normal)
        .value("Highlighted", MessageOut::Highlighted)
        .value("Warning", MessageOut::Warning)
        .value("Error", MessageOut::Error)
        .export_values();

    messageOut
        .def_property_readonly_static("master", [](py::object){ return MessageOut::master(); })
        .def_property_readonly_static("interactive", [](py::object){ return MessageOut::interactive(); })
        .def("put", (void(MessageOut::*)(const std::string&, int))&MessageOut::put,
             py::arg("message"), py::arg("type") = (int)MessageOut::Normal)
        .def("putln", (void(MessageOut::*)(const std::string&, int))&MessageOut::putln,
             py::arg("message"), py::arg("type") = (int)MessageOut::Normal)
        .def("notify", (void(MessageOut::*)(const std::string&, int))&MessageOut::notify,
             py::arg("message"), py::arg("type") = (int)MessageOut::Normal)
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
