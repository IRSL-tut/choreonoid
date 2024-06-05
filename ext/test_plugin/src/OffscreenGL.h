#ifndef __OFFSCREEN_GL_H__
#define __OFFSCREEN_GL_H__

#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

#include <iostream>

class OffscreenGL : public QOpenGLFunctions
{
public:
    OffscreenGL() {};
    QOffscreenSurface *surface = nullptr;
    QOpenGLContext    *context = nullptr;
    bool create() {
        QSurfaceFormat _fmt;
        setDefaultSurfaceFormat(_fmt);
        return create(_fmt);
    }
    bool create(const QSurfaceFormat &_fmt) {
        context = new QOpenGLContext();
        context->setFormat(_fmt);
        bool res = context->create();
        if (res) {
            surface = new QOffscreenSurface();
            surface->setFormat(context->format());
            surface->create();
            ///
            context->makeCurrent(surface);
            initializeOpenGLFunctions();
        }
        return res;
    }
    bool makeCurrent() {
        if (!!surface && !!context) {
            return context->makeCurrent(surface);
        }
        return false;
    }
    bool makeBuffer(unsigned int nWidth, unsigned int nHeight,
                    unsigned int* uiResolveTextureId, unsigned int *uiResolveFramebufferId) {
        glGenFramebuffers(1, uiResolveFramebufferId );
        glBindFramebuffer(GL_FRAMEBUFFER, *uiResolveFramebufferId);
        //
        glGenTextures(1, uiResolveTextureId );
        glBindTexture(GL_TEXTURE_2D, *uiResolveTextureId );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *uiResolveTextureId, 0);

        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        bool res = true;
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            res = false;
        }
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer( GL_FRAMEBUFFER, 0 );
        return res;
    }

    bool writeTexture(unsigned int textureId, unsigned char *buffer, unsigned int nWidth, unsigned int nHeight,
                      int offsetWidth, int offsetHeight) {
        glBindTexture( GL_TEXTURE_2D, textureId );
        //test.glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_WIDTH , &gl_w );
        //test.glGetTexLevelParameteriv( GL_TEXTURE_2D , 0 , GL_TEXTURE_HEIGHT, &gl_h );
        glTexSubImage2D( GL_TEXTURE_2D, 0,
                         offsetWidth, offsetHeight, nWidth, nHeight,
                         GL_RGB, GL_UNSIGNED_BYTE, buffer );
        glBindTexture( GL_TEXTURE_2D, 0 );
        return true;
    }
    static void setDefaultSurfaceFormat(QSurfaceFormat &_fmt) {
        _fmt.setVersion(4, 6);
        _fmt.setRedBufferSize(8);//
        _fmt.setBlueBufferSize(8);//
        _fmt.setGreenBufferSize(8);//
        _fmt.setAlphaBufferSize(8);//
        //_fmt.setStencilBufferSize(0);//
        _fmt.setDepthBufferSize(24);
        _fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
        //_fmt.setProfile(QSurfaceFormat::CoreProfile);
        _fmt.setRenderableType(QSurfaceFormat::OpenGL);
        _fmt.setOption(QSurfaceFormat::DeprecatedFunctions);
        QSurfaceFormat::setDefaultFormat(_fmt);
    }
    static void printSurfaceFormat(const QSurfaceFormat &_format, std::ostream &os_)
    {
        //// lambda functions
        auto formatOptionsToString = [] (QSurfaceFormat::FormatOptions _value) -> std::string
        {
            std::string options;
            if (_value & QSurfaceFormat::StereoBuffers) {
                options.append("StereoBuffers");
            }
            if (_value & QSurfaceFormat::DebugContext) {
                options.empty() ? options.append("") : options.append(", ");
                options.append("DebugContext");
            }
            if (_value & QSurfaceFormat::DeprecatedFunctions) {
                options.empty() ? options.append("") : options.append(", ");
                options.append("DeprecatedFunctions");
            }
            if (_value & QSurfaceFormat::ResetNotification) {
                options.empty() ? options.append("") : options.append(", ");
                options.append("ResetNotification");
            }
            return options;
        };
        auto openGLContextProfileToString = [] (QSurfaceFormat::OpenGLContextProfile _value) -> std::string
        {
            switch (_value) {
            case QSurfaceFormat::NoProfile:
                return "NoProfile";
            case QSurfaceFormat::CoreProfile:
                return "CoreProfile";
            case QSurfaceFormat::CompatibilityProfile:
                return "CompatibilityProfile";
            default:
                return "Invalid OpenGLContextProfile";
            }
        };
        auto renderableTypeToString = [] (QSurfaceFormat::RenderableType _value) -> std::string
        {
            switch (_value) {
            case QSurfaceFormat::DefaultRenderableType:
                return "DefaultRenderableType";
            case QSurfaceFormat::OpenGL:
                return "OpenGL";
            case QSurfaceFormat::OpenGLES:
                return "OpenGLES";
            case QSurfaceFormat::OpenVG:
                return "OpenVG";
            default:
                return "Invalid RenderableType";
            }
        };
        auto swapBehaviorToString = [] (QSurfaceFormat::SwapBehavior _value) -> std::string
        {
            switch (_value) {
            case QSurfaceFormat::DefaultSwapBehavior:
                return "DefaultSwapBehavior";
            case QSurfaceFormat::SingleBuffer:
                return "SingleBuffer";
            case QSurfaceFormat::DoubleBuffer:
                return "DoubleBuffer";
            default:
                return "Invalid SwapBehavior";
            }
        };

        // surface format info
        os_ << "version: "
            << _format.version().first << "."
            << _format.version().second << std::endl;
        os_ << "profile: "
            << openGLContextProfileToString(_format.profile()) << std::endl;
        os_ << "options: "
            << formatOptionsToString(_format.options()) << std::endl;
        os_ << "renderableType: "
            << renderableTypeToString(_format.renderableType()) << std::endl;
        os_ << "hasAlpha: " << _format.hasAlpha() << std::endl;
        os_ << "redBufferSize: " << _format.redBufferSize() << std::endl;
        os_ << "greenBufferSize: " << _format.greenBufferSize() << std::endl;
        os_ << "blueBufferSize: " << _format.blueBufferSize() << std::endl;
        os_ << "alphaBufferSize: " << _format.alphaBufferSize() << std::endl;
        os_ << "depthBufferSize: " << _format.depthBufferSize() << std::endl;
        os_ << "stencilBufferSize: " << _format.stencilBufferSize() << std::endl;
        os_ << "samples: " << _format.samples() << std::endl;
        os_ << "swapBehavior: "
            << swapBehaviorToString(_format.swapBehavior()) << std::endl;
        os_ << "swapInterval: " << _format.swapInterval() << std::endl;
        os_ << std::endl;
    }
};

#endif
