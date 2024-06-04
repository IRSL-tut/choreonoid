#include "TestPlugin.h"

#include <fmt/format.h>
#define IRSL_DEBUG
#include "irsl_debug.h"

#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/GLSLSceneRenderer>

#include <cnoid/App>
#include <cnoid/Timer>

using namespace cnoid;

namespace {
TestPlugin* instance_ = nullptr;
}

//// Impl
class TestPlugin::Impl
{
public:
    Impl();
    void initialize();
    void singleLoop();

public:
    Timer tm;
    long counter;
    double publishingRate;
};

TestPlugin::Impl::Impl()
{
}
void TestPlugin::Impl::initialize()
{
    SceneWidget *sw = SceneView::instance()->sceneWidget();
    GLSceneRenderer *sr = sw->renderer<GLSceneRenderer>();

    GLSLSceneRenderer *slsr = static_cast<GLSLSceneRenderer *>(sr);
    if (!!slsr) {
        DEBUG_STREAM(" GLSL");
        unsigned int uiResolveTextureId;
        unsigned int uiResolveFramebufferId;
        std::cerr << "aa" << std::endl;
        bool res = slsr->makeBuffer(1200, 800, &uiResolveTextureId, &uiResolveFramebufferId);
        DEBUG_STREAM(" GLSL: res: " << res);
    }

    tm.sigTimeout().connect( [this]() { this->singleLoop(); });

    int interval_ms = 1000/33;

    tm.start(interval_ms);
}
void TestPlugin::Impl::singleLoop()
{
    DEBUG_PRINT();
}

TestPlugin* TestPlugin::instance()
{
    return instance_;
}

////
TestPlugin::TestPlugin()
    : Plugin("Test")
{
    instance_ = this;
    impl = new Impl();
}
TestPlugin::~TestPlugin()
{
}

bool TestPlugin::initialize()
{
    DEBUG_PRINT();

    App::sigExecutionStarted().connect( [this]() {
                                            impl->initialize();
                                        });
    return true;
}

bool TestPlugin::finalize()
{
    DEBUG_PRINT();
    return true;
}

const char* TestPlugin::description() const
{
    static std::string text =
        fmt::format("Test Plugin Version {}\n", CNOID_FULL_VERSION_STRING) +
        "\n" +
        "Copyrigh (c) 2022 IRSL-tut Development Team.\n"
        "\n" +
        MITLicenseText() +
        "\n"  ;

    return text.c_str();
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(TestPlugin);
