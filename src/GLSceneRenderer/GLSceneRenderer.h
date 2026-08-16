#ifndef CNOID_BASE_GL_SCENE_RENDERER_H
#define CNOID_BASE_GL_SCENE_RENDERER_H

#include <cnoid/SceneRenderer>
#include <cnoid/gl.h>
#include "exportdecl.h"

namespace cnoid {

class Image;
class ShaderProgram;
class LightingProgram;

/**
   The scene renderer implemented with the OpenGL shading language.

   Note that some of the members of this class are not specific to OpenGL and
   would also be required by the renderers based on the other graphics APIs
   such as Vulkan and Filament. Those members are marked with the comment
   "API-independent" so that they can be moved up to the SceneRenderer class
   when another renderer implementation is introduced. The actual interface
   should be determined by the requirements of that implementation, so the
   members are kept here until then.
*/
class CNOID_EXPORT GLSceneRenderer : public SceneRenderer
{
public:
    static GLSceneRenderer* create(SgGroup* root = nullptr);

    /**
       The rendering mode of transparent objects. This is a system-wide setting
       shared by all the renderer instances including the ones used for the
       vision sensor simulation. In the sorted rendering mode, transparent
       objects are rendered by the alpha blending in back-to-front sorted
       order, which cannot correctly handle intersecting objects. In the depth
       peeling mode, the overlaps of transparent objects are resolved pixel by
       pixel, which gives the correct rendering result even for intersecting
       objects. In the supersampled depth peeling mode, the transparent object
       layers are additionally rendered at doubled resolution and downsampled
       in the compositing so that the silhouettes of the transparent objects
       are anti-aliased at the cost of the additional GPU memory and fill rate.

       API-independent.
    */
    enum TransparentRenderingMode {
        SortedTransparentRendering,
        DepthPeelingTransparentRendering,
        SupersampledDepthPeelingTransparentRendering,
        NumTransparentRenderingModes
    };
    static void setTransparentRenderingMode(int mode);
    static int transparentRenderingMode();
    static SignalProxy<void()> sigTransparentRenderingModeChanged();

    /**
       The maximum number of the transparent object layers extracted by the depth
       peeling. The transparent surfaces deeper than this number at a pixel are
       not rendered. A larger number improves the rendering of the deeply
       overlapped transparent objects at the cost of the rendering passes, each
       of which renders all the transparent objects again. This is a system-wide
       setting shared by all the renderer instances in the same way as the
       transparent rendering mode. The value is clipped to the range between
       MinNumDepthPeelingLayers and MaxNumDepthPeelingLayers.

       API-independent.
    */
    enum { MinNumDepthPeelingLayers = 1, MaxNumDepthPeelingLayers = 8 };
    static void setNumDepthPeelingLayers(int n);
    static int numDepthPeelingLayers();
    static SignalProxy<void()> sigNumDepthPeelingLayersChanged();

    GLSceneRenderer(SgGroup* root = nullptr);
    virtual ~GLSceneRenderer();

    static void addExtension(std::function<void(GLSceneRenderer* renderer)> func);
    virtual void applyExtensions() override;
    virtual bool applyNewExtensions() override;

    virtual void setOutputStream(std::ostream& os);

    virtual SgGroup* sceneRoot() override;
    virtual SgGroup* scene() override;

    virtual PolymorphicSceneNodeFunctionSet* renderingFunctions() override;
    virtual void renderCustomGroup(SgGroup* transform, const std::function<void()>& traverseFunction) override;
    virtual void renderCustomTransform(SgTransform* transform, const std::function<void()>& traverseFunction) override;
    virtual void renderNode(SgNode* node) override;

    virtual void invalidatePlotVertices(SgPlot* plot) override;

    virtual void addNodeDecoration(SgNode* node, NodeDecorationFunction func, int id) override;
    virtual void clearNodeDecorations(int id) override;

    virtual const Affine3& currentModelTransform() const override;
    virtual const Matrix4& projectionMatrix() const override;
    virtual const Matrix4& viewProjectionMatrix() const override;
    virtual Vector3 project(const Vector3& p) const override;
    virtual double projectedPixelSizeRatio(const Vector3& position) const override;

    const Isometry3& viewTransform() const;
    Matrix4 modelViewMatrix() const;
    Matrix4 modelViewProjectionMatrix() const;

    void pushShaderProgram(ShaderProgram* program);
    void popShaderProgram();

    void renderLights(LightingProgram* program);
    void renderFog(LightingProgram* program);

    void dispatchToTransparentPhase(
        ReferencedPtr object, int id,
        const std::function<void(Referenced* object, const Affine3& modelTransform, int id)>& renderingFunction);

    /**
       The base class of the GPU resources managed by the renderer.

       A resource is released in one of the two following ways, and the
       subclasses must implement both:

       - The destructor is called when the renderer releases the resource in
         the normal way, that is at the end of a rendering pass in which the
         resource was not used, or when the resource map is explicitly
         cleared. The corresponding GL context is current then, so the
         destructor should delete the GL objects the resource owns.

       - discard() is called when the GL context has been lost or cannot be
         made current. The implementation must just forget the GL object
         handles without deleting them, so that the destructor called
         afterwards does not touch the invalid handles.
    */
    class CNOID_EXPORT GLResource : public Referenced
    {
    public:
        virtual void discard() = 0;
    };

    typedef ref_ptr<GLResource> GLResourcePtr;

    /**
       Returns the GPU resource associated with the given key object, or
       creates it with the given factory function when the association does
       not exist yet. The resource is managed by the renderer: it is released
       at the end of a rendering pass in which it was not used, and it is
       correctly discarded when the GL context is cleared. An extension that
       keeps GPU resources per scene object in its custom rendering function
       can use this instead of managing the resource lifetimes by itself.

       This function is only valid during rendering; call it from a rendering
       function. Note that the standard rendering associates its own resource
       with the drawable scene objects using the same map, so the key must be
       a dedicated object that the standard rendering never keys, such as a
       proxy object held by the node for this purpose.
    */
    GLResource* getOrCreateGLResource(
        SgObject* key, const std::function<GLResourcePtr()>& factory);

    virtual bool initializeGL(GLADloadfunc getProcAddress);
    virtual void flushGL();

    /**
       This function clears all the OpenGL resourses used in the renderer.
       The function should be called when the renderer is deleted.
       The function must be called when the OpenGL context is changed, and
       then the initializeGL function must be called again for the new OpenGL
       context. Note that the corresponding OpenGL context must be made current
       when the function is called.
    */
    virtual void clearGL();

    virtual void setDefaultFramebufferObject(unsigned int id);

    virtual const std::string& glVersionString() const;
    virtual const std::string& glslVersionString() const;
    virtual const std::string& glVendorString() const;
    virtual const std::string& glRendererString() const;

    virtual void setViewport(int x, int y, int width, int height);

    //! Call this function when the OpenGL viewport is updated by the system.
    virtual void updateViewportInformation();

    //! Call this function instead of setViewport when the viewport is specified by the system.
    virtual void updateViewportInformation(int x, int y, int width, int height);

    //! API-independent.
    struct Viewport {
        int x;
        int y;
        int w;
        int h;
    };

    //! API-independent.
    const Viewport& viewport() const { return viewport_; }
    virtual float devicePixelRatio() const override;
    void setDevicePixelRatio(float r){ devicePixelRatio_ = r; }

    /**
       The following projection matrix functions are pure mathematics and are
       API-independent.
    */
    void getPerspectiveProjectionMatrix(
        double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix);
    void getOrthographicProjectionMatrix(
        double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix);
    void getReversedPerspectiveProjectionMatrix(
        double fovy, double aspect, double zNear, double zFar, Matrix4& out_matrix);
    void getReversedInfinitePerspectiveProjectionMatrix(
        double fovy, double aspect, double zNear, Matrix4& out_matrix);
    void getReversedOrthographicProjectionMatrix(
        double left,  double right,  double bottom,  double top,  double nearVal,  double farVal, Matrix4& out_matrix);

    //! API-independent.
    void getViewFrustum(const SgPerspectiveCamera* camera, double& left, double& right, double& bottom, double& top) const;
    //! API-independent.
    void getViewVolume(const SgOrthographicCamera* camera, float& out_left, float& out_right, float& out_bottom, float& out_top) const;

    //! API-independent except for the NDC depth range switched by isReversedDepthBuffer.
    virtual bool unproject(double x, double y, double z, Vector3& out_projected) const override;
    virtual bool getCameraRay(double x, double y, Vector3& out_origin, Vector3& out_direction) const override;

    //! API-independent.
    const Vector3f& backgroundColor() const;
    //! API-independent.
    void setBackgroundColor(const Vector3f& color);
    //! API-independent.
    const Vector3f& defaultColor() const;
    virtual void setDefaultColor(const Vector3f& color);

    //! API-independent.
    enum LightingMode {
        NormalLighting,
        MinimumLighting,
        SolidColorLighting,
        NoLighting,
        NumLightingModes
    };
    virtual void setLightingMode(LightingMode mode);
    virtual LightingMode lightingMode() const;

    virtual bool isShadowCastingAvailable() const;
    virtual void setWorldLightShadowEnabled(bool on = true);
    virtual void setAdditionalLightShadowEnabled(int index, bool on = true);
    virtual void clearAdditionalLightShadows();
    virtual void setShadowAntiAliasingEnabled(bool on);
    virtual void setShadowMapSize(int width, int height);

    virtual void setDefaultSmoothShading(bool on);
    virtual SgMaterial* defaultMaterial();
    virtual void enableTexture(bool on);
    virtual void setMaterialAmbientNormalizationEnabled(bool on);
    virtual void setDefaultPointSize(double size);
    virtual void setDefaultLineWidth(double width);
    virtual void setNormalVisualizationEnabled(bool on);
    virtual void setNormalVisualizationLength(double length);
    virtual void requestToClearResources();
    virtual void enableUnusedResourceCheck(bool on);
    virtual const Vector3& pickedPoint() const;
    virtual const SgNodePath& pickedNodePath() const;
    virtual bool isRenderingPickingImage() const override;
    virtual void setColor(const Vector3f& color);
    virtual void setUpsideDown(bool on);

    virtual void setMsaaLevel(int level);
    virtual int msaaLevel() const;
    virtual void setDepthBufferUpdateEnabled(bool on);
    virtual bool isDepthBufferUpdateEnabled() const;

    //! API-independent.
    enum CullingMode {
        EnableBackFaceCulling,
        DisableBackFaceCulling,
        ForceBackfaceCulling,
        NumCullingModes,

        // deprecated
        ENABLE_BACK_FACE_CULLING = EnableBackFaceCulling,
        DISABLE_BACK_FACE_CULLING = DisableBackFaceCulling,
        FORCE_BACK_FACE_CULLING = ForceBackfaceCulling,
        N_CULLING_MODES = NumCullingModes
    };

    virtual void setBackFaceCullingMode(int mode);
    virtual int backFaceCullingMode() const;

    static void forceStandardDepthBuffer();
    static bool isStandardDepthBufferForced();

    virtual bool isReversedDepthBuffer() const;
    virtual void setInfiniteFarOverrideEnabled(bool on);
    virtual bool isInfiniteFarOverrideEnabled() const;

    virtual void setBoundingBoxRenderingForLightweightRenderingGroupEnabled(bool on);

    void setLowMemoryConsumptionMode(bool on);

    virtual void setPickingImageOutputEnabled(bool on);
    virtual bool getPickingImage(Image& out_image);

    [[deprecated("Use setNormalVisualizationEnabled and setNormaliVisualizationLength.")]]
    void showNormalVectors(double length);

    class Impl;

protected:
    virtual void doRender() override;
    virtual bool doPick(int x, int y) override;
    virtual void onHighlightColorChanged() override;

private:
    /**
       The base parts of the corresponding public functions. They are separated
       so that the overriding implementations can call them without recursion.
    */
    void setOutputStreamBase(std::ostream& os);
    void setDefaultColorBase(const Vector3f& color);
    void updateViewportInformationBase(int x, int y, int width, int height);

    Viewport viewport_;
    float devicePixelRatio_;

    Impl* impl;
};

/**
   \deprecated Use GLSceneRenderer. This alias is kept for the compatibility
   with the existing code that refers to the renderer by this name.
*/
using GLSLSceneRenderer = GLSceneRenderer;

}

#endif
