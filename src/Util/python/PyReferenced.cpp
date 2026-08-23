#include "PyUtil.h"
#include "PyReferenced.h"

namespace nb = nanobind;
using namespace cnoid;

namespace {

// incref/decref for a binding-language-owned Referenced object, delegating to
// the bound Python wrapper's reference count. They acquire the GIL because
// addRef()/releaseRef() may be called from any C++ thread.
//
// The decref guards against a finalized interpreter with nb::is_alive(): once the
// Python runtime has shut down, touching it (Py_DECREF) would crash. In that case
// we skip the decref entirely; the object then leaks (its memory and destructor
// are left to process teardown), which is the accepted trade-off for objects
// still referenced from C++ at interpreter exit. See the long comment in
// Referenced.h.
void referencedIncref(void* wrapper) noexcept
{
    nb::gil_scoped_acquire guard;
    Py_INCREF(static_cast<PyObject*>(wrapper));
}

void referencedDecref(void* wrapper) noexcept
{
    if(!nb::is_alive()){
        return;
    }
    nb::gil_scoped_acquire guard;
    Py_DECREF(static_cast<PyObject*>(wrapper));
}

}

namespace cnoid {

void exportPyReferenced(nb::module_& m)
{
    // Install the reference-counting bridge used once an object is in
    // binding-language mode.
    initReferencedPythonInterface({ referencedIncref, referencedDecref });

    // Register Referenced with nanobind's intrusive ownership. The set_self_py
    // hook (calling setSelfPython) is inherited by every Referenced-derived type
    // and hands ownership to the Python wrapper on first exposure.
    nb::class_<Referenced>(
        m, "Referenced",
        nb::intrusive_ptr<Referenced>(
            [](Referenced* p, PyObject* self) noexcept {
                void* primary = p->selfPython();
                if(!primary){
                    p->setSelfPython(self);
                } else if(primary != self){
                    /*
                      nanobind has created a second wrapper for an object that is
                      already owned by another wrapper. This happens when the two
                      exposures use different static types neither of which is a
                      Python subtype of the other, e.g. an item whose dynamic type
                      is not bound is first exposed as Item (via an ItemList) and
                      later returned as SimulatorItem*; nb_type_put() then cannot
                      reuse the Item wrapper. The new wrapper was created with
                      take_ownership (destruct/cpp_delete set) but setSelfPython()
                      cannot accept it - the object can delegate its reference
                      count to only one wrapper - so if left as is, both wrappers
                      would destroy the C++ object, and this one without holding
                      the C++-side reference count (a use-after-free / double
                      free). Demote it to a non-owning alias: clear its ownership
                      flags and pin the primary wrapper for the alias's lifetime
                      so the C++ object cannot die under it.
                    */
                    nb::inst_set_state(self, true, false);
                    nb::detail::keep_alive(self, static_cast<PyObject*>(primary));
                }
            }));
}

}
