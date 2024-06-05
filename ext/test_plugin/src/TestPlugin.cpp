#include <fmt/format.h>
#define IRSL_DEBUG
#include "irsl_debug.h"

#include <cnoid/SceneWidget>
#include <cnoid/SceneView>
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/MessageView>
#include <cnoid/App>
#include <cnoid/Timer>

#include "TestPlugin.h"
#include "OffscreenGL.h"

#include <openvr.h>

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
    unsigned long counter;
    double publishingRate;

    OffscreenGL offGL;

    vr::IVRSystem *m_pHMD;
    GLSLSceneRenderer *slsr;
    unsigned int nWidth, nHeight;
    unsigned int uiResolveTextureId;
    unsigned int uiResolveFramebufferId;

    std::ostream *os_;
};

TestPlugin::Impl::Impl()
{
    m_pHMD = nullptr;
    slsr = nullptr;
    os_ = nullptr;
    counter = 0;
}
void TestPlugin::Impl::initialize()
{
    os_ = &(MessageView::instance()->cout(false));

    //SceneWidget *sw = SceneView::instance()->sceneWidget();
    //GLSceneRenderer *sr = sw->renderer<GLSceneRenderer>();
    //slsr = static_cast<GLSLSceneRenderer *>(sr);

    vr::EVRInitError eError = vr::VRInitError_None;
    m_pHMD = vr::VR_Init( &eError, vr::VRApplication_Scene );
    if ( eError != vr::VRInitError_None ) {
        m_pHMD = NULL;
        //printf( "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription( eError ) );
        *os_ << "Unable to init VR runtime:  " << vr::VR_GetVRInitErrorAsEnglishDescription( eError ) << std::endl;
        return;
    }
    m_pHMD->GetRecommendedRenderTargetSize( &nWidth, &nHeight );
    *os_ << "width x height = " << nWidth << " x  " << nHeight << std::endl;

    vr::HmdMatrix44_t l_mat = m_pHMD->GetProjectionMatrix( vr::Eye_Left,  0.01f, 15.0f );
    vr::HmdMatrix44_t r_mat = m_pHMD->GetProjectionMatrix( vr::Eye_Right, 0.01f, 15.0f );
    vr::HmdMatrix34_t l_eye = m_pHMD->GetEyeToHeadTransform( vr::Eye_Left );
    vr::HmdMatrix34_t r_eye = m_pHMD->GetEyeToHeadTransform( vr::Eye_Right );

    {
        bool glres = offGL.create();
        *os_ << "offGL: create: " << glres << std::endl;
        glres = offGL.makeCurrent();
        *os_ << "offGL: current: " << glres << std::endl;
        glres = offGL.makeBuffer(nWidth, nHeight, &uiResolveTextureId, &uiResolveFramebufferId);
        *os_ << "offGL: buffer: " << glres << std::endl;

        OffscreenGL::printSurfaceFormat(offGL.context->format(), *os_);
    }
    offGL.makeCurrent();
    if ( !vr::VRCompositor() ) {
        *os_ << "Compositor initialization failed. See log file for details" << std::endl;
        return;
    }
    tm.sigTimeout().connect( [this]() { this->singleLoop(); });

    int interval_ms = 1000/5;

    tm.start(interval_ms);
}
void TestPlugin::Impl::singleLoop()
{
    DEBUG_PRINT();
    if (!!m_pHMD) {
        //SceneWidget *sw = SceneView::instance()->sceneWidget();
        //sw->makeCurrent();
        std::vector<unsigned char> img(nWidth * nHeight * 3);
        unsigned char *ptr = img.data();
        unsigned char ret = counter++ % 0xFF;

        for(int i = 0; i < nHeight; i++) {
            for(int j = 0; j < nWidth; j++) {
                ptr[3*(i*nWidth + j)] = 0xFF;
                ptr[3*(i*nWidth + j)+1] = 0;
                ptr[3*(i*nWidth + j)+2] = 0;
            }
        }
        ////
        *os_ << "up" << std::endl;
        ////
        offGL.makeCurrent();

        vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
        vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

        offGL.writeTexture(uiResolveTextureId, ptr, nWidth, nHeight, 0, 0);

        vr::Texture_t leftEyeTexture =  {(void*)(uintptr_t)uiResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left,  &leftEyeTexture );

        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)uiResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );

        //// twice
        vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
        //vr::Compositor_FrameTiming tmg;
        //bool tm_q = vr::VRCompositor()->GetFrameTiming(&tmg);
    }
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
