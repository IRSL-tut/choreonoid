#include "../LazyCaller.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace {

// Wraps a Python callable so that it can be invoked later from C++ (callLater /
// callSynchronously). An error raised on the Python side is reported through
// sys.unraisablehook instead of propagating a C++ exception into the GUI event
// loop, where it would crash the process. This matches the exception-safe slot
// handling used for signals (see Util's PySignal.h).
struct PyFunc
{
    nb::object func;

    PyFunc(nb::object f) : func(std::move(f)) { }

    /*
       The GIL must be acquired when the reference count of the held Python object
       is changed. Note that this object is copied and destroyed on the C++ side
       where the GIL is not necessarily held. In particular, a pending call event
       can be destroyed during the application shutdown.
    */
    PyFunc(const PyFunc& org) {
        nb::gil_scoped_acquire lock;
        func = org.func;
    }

    PyFunc(PyFunc&& org) noexcept : func(std::move(org.func)) { }

    ~PyFunc() {
        if(func.is_valid()){
            nb::gil_scoped_acquire lock;
            func.reset();
        }
    }

    void operator()() {
        nb::gil_scoped_acquire lock;
        try {
            func();
        } catch(nb::python_error& ex) {
            ex.discard_as_unraisable("Choreonoid lazy caller");
        }
    }
};

}

namespace cnoid {

void exportPyLazyCaller(nb::module_ m)
{
    nb::class_<LazyCaller> lazyCaller(m, "LazyCaller");

    nb::enum_<LazyCaller::Priority>(lazyCaller, "Priority")
        .value("HighPriority", LazyCaller::HighPriority)
        .value("NormalPriority", LazyCaller::NormalPriority)
        .value("LowPriority", LazyCaller::LowPriority)
        .value("MinimumPriority", LazyCaller::MinimumPriority)
        ;

    m.def("callLater",
          [](nb::object func, int priority){
              cnoid::callLater(PyFunc(func), priority); },
          nb::arg("func"), nb::arg("priority") = (int)LazyCaller::NormalPriority);

    m.def("callSynchronously",
          [](nb::object func, int priority){
              cnoid::callSynchronously(PyFunc(func), priority); },
          nb::arg("func"), nb::arg("priority") = (int)LazyCaller::NormalPriority);
}

}
