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
    void updatePoses();
    bool getDeviceString(std::string &_res, int index, vr::TrackedDeviceProperty prop);

public:
    Timer tm;
    unsigned long counter;
    double publishingRate;

    OffscreenGL offGL;

    vr::IVRSystem *m_pHMD;
    //GLSLSceneRenderer *slsr;
    unsigned int nWidth, nHeight;
    unsigned int ui_L_TextureId;
    unsigned int ui_L_FramebufferId;
    unsigned int ui_R_TextureId;
    unsigned int ui_R_FramebufferId;

    std::ostream *os_;

    vr::TrackedDevicePose_t TrackedDevicePoses[ vr::k_unMaxTrackedDeviceCount ];
    // Eigen // devicePoses
    //std::vector<> devicePoses
    std::vector<std::string> deviceNames;
    std::vector<int> deviceClasses;
};

//// >>>> Impl
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
        glres = offGL.makeBuffer(nWidth, nHeight, &ui_R_TextureId, &ui_R_FramebufferId);
        *os_ << "offGL: buffer: " << glres << std::endl;
        glres = offGL.makeBuffer(nWidth, nHeight, &ui_L_TextureId, &ui_L_FramebufferId);
        *os_ << "offGL: buffer: " << glres << std::endl;
        offGL.glFinish();
        offGL.glFlush();
        offGL.context->doneCurrent();
        OffscreenGL::printSurfaceFormat(offGL.context->format(), *os_);
    }

    offGL.makeCurrent();
    if ( !vr::VRCompositor() ) {
        *os_ << "Compositor initialization failed. See log file for details" << std::endl;
        return;
    }
    offGL.glFinish();
    offGL.glFlush();
    offGL.context->doneCurrent();

    tm.sigTimeout().connect( [this]() { this->singleLoop(); });

    int interval_ms = 1000/30;

    tm.start(interval_ms);
}

void TestPlugin::Impl::singleLoop()
{
    DEBUG_PRINT();
    if (!!m_pHMD) {
        std::vector<unsigned char> img_R(nWidth * nHeight * 3);
        std::vector<unsigned char> img_L(nWidth * nHeight * 3);
        unsigned char *ptr_R = img_R.data();
        unsigned char *ptr_L = img_L.data();
        unsigned char col_R = counter++ % 0xFF;
        unsigned char col_L = 0xFF - col_R;
        for(int i = 0; i < nHeight; i++) {
            for(int j = 0; j < nWidth; j++) {
                ptr_R[3*(i*nWidth + j)]   = col_R;
                ptr_R[3*(i*nWidth + j)+1] = 0;
                ptr_R[3*(i*nWidth + j)+2] = 0;
                ptr_L[3*(i*nWidth + j)]   = 0;
                ptr_L[3*(i*nWidth + j)+1] = 0;
                ptr_L[3*(i*nWidth + j)+2] = col_L;
            }
        }
        ////
        offGL.makeCurrent();
        offGL.writeTexture(ui_R_TextureId, ptr_R, nWidth, nHeight, 0, 0);
        offGL.writeTexture(ui_L_TextureId, ptr_L, nWidth, nHeight, 0, 0);
        vr::Texture_t leftEyeTexture =  {(void*)(uintptr_t)ui_L_TextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        auto resL = vr::VRCompositor()->Submit(vr::Eye_Left,  &leftEyeTexture );
        vr::Texture_t rightEyeTexture = {(void*)(uintptr_t)ui_R_TextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
        auto resR = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture );
        if (resL != 0) {
            *os_ << "L: " << resL << std::endl;
        }
        if (resR != 0) {
            *os_ << "R: " << resR << std::endl;
        }
        updatePoses();
        //vr::TrackedDevicePose_t m_rTrackedDevicePose[ vr::k_unMaxTrackedDeviceCount ];
        //vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
        //vr::Compositor_FrameTiming tmg;
        //bool tm_q = vr::VRCompositor()->GetFrameTiming(&tmg);
    }
}

bool TestPlugin::Impl::getDeviceString(std::string &_res, int index, vr::TrackedDeviceProperty prop)
{
    // prop
    //vr::Prop_RenderModelName_String
    //vr::Prop_TrackingSystemName_String
    //vr::Prop_SerialNumber_String
    vr::TrackedPropertyError p_error;
    uint32_t len = m_pHMD->GetStringTrackedDeviceProperty( index, prop, NULL, 0, &p_error );
    if( len == 0 ) {
        return false;
    }
    char *buf_ = new char[ len ];
    len = m_pHMD->GetStringTrackedDeviceProperty( index, prop, buf_, len, &p_error );
    _res = buf;
    delete [] buf_;
    return true;
}

void TestPlugin::Impl::updatePoses()
{
    if ( !m_pHMD ) {
        return;
    }

    vr::VRCompositor()->WaitGetPoses(TrackedDevicePoses, vr::k_unMaxTrackedDeviceCount, NULL, 0 );

    int validPoseCount = 0;
    for ( int idx = 0; idx < vr::k_unMaxTrackedDeviceCount; idx++ ) {
        int cls = m_pHMD->GetTrackedDeviceClass(idx);
        if ( m_rTrackedDevicePose[idx].bPoseIsValid ) {
            validPoseCount++;
            deviceClasses[idx] = cls;
#if 0
            // store poses
            if (device_poses_[index].bDeviceIsConnected &&
                device_poses_[index].eTrackingResult == vr::TrackingResult_Running_OK) {
                ///
            }
#endif

        }
        if(cls == 0) {
            // do nothing
            break;
        }
        if(cls == vr::TrackedDeviceClass_HMD) {
            // update HMD pose
            break;
        }
        if(cls == vr::TrackedDeviceClass_Controller) {
            // update controller pose

            // update controller state(button etc.)
            vr::VRControllerState_t state;
            m_pHMD->GetControllerState(idx, &state, sizeof(vr::VRControllerState_t));

            break;
        }
        if(cls == vr::TrackedDeviceClass_GenericTracker) {
            break;
        }
        if(cls == vr::TrackedDeviceClass_TrackingReference) {
            break;
        }
    }

#if 0
    if ( m_rTrackedDevicePose[ vr::k_unTrackedDeviceIndex_Hmd ].bPoseIsValid ) {
        //m_mat4HMDPose = m_rmat4DevicePose[ vr::k_unTrackedDeviceIndex_Hmd ];
        //m_mat4HMDPose.invert();
    }
#endif

    vr::VREvent_t event;
    while( m_pHMD->PollNextEvent( &event, sizeof( event ) ) ) {
        switch( event.eventType ) {
        case vr::VREvent_TrackedDeviceDeactivated:
            break;
        case vr::VREvent_TrackedDeviceUpdated:
            break;
        }
    }
}
//// <<<< Impl

////
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
